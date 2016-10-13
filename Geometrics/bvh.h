#pragma once
//#include "DirectXMathExtend.h"
#include <vector>
#include <algorithm>
#include <minmax>
#include <iterator>
#include <queue>
#include <stack>
#include <functional>

// We need AlignedBox 
#include <iterator_range.h>

namespace Geometrics {
	using ::std::iterator_range;

	static constexpr size_t _align_boundry = 16U;

	//concept ExtBVH
	//{
	//  Typedefs:
	//	typedef Index;
	//	typedef Object;
	//	typedef Volume;
	//	typedef VolumeIterator;
	//  (*VolumeIterator == Index)
	//	typedef ObjectIterator;
	//  
	//  Interfaces:
	//	Index getRootIndex() const;
	//	bool isObject(Index index) const;
	//	const Object& getObject(Index index) const;
	//	const Volume& getVolume(Index index) const;
	//	bool getChildren(Index index, VolumeIterator &outVBegin, VolumeIterator &outVEnd) const;
	//};

	namespace internal {
		template<typename _Scalar, size_t _Dim>
		struct aabb_t
		{
			using scalar_t = _Scalar;
			static constexpr size_t Dim = _Dim;
			//static constexpr size_t _ArraySize = ((Dim * sizeof(scalar_t) - 1) / _align_boundry + 1) * _align_boundry;
			using vector_t = scalar_t[Dim];
			using this_type = aabb_t<_Scalar, _Dim>;

			alignas(_align_boundry)vector_t min;
			alignas(_align_boundry)vector_t max;

			template <size_t dim>
			static bool less(const this_type& lhs, const this_type & rhs)
			{
				return lhs.min[dim] + lhs.max[dim] < rhs.min[dim] + rhs.max[dim];
			}

			this_type& operator += (const this_type& rhs)
			{
				for (int i = 0; i < Dim; i++)
					min[i] = std::min(min[i], rhs.min[i]);
				for (int i = 0; i < Dim; i++)
					max[i] = std::max(max[i], rhs.max[i]);
			}

			// bad practice
			this_type operator+(const this_type& rhs)
			{
				this_type& lhs = *this;
				this_type ret;
				for (int i = 0; i < Dim; i++)
					ret.min[i] = std::min(lhs.min[i], rhs.min[i]);
				for (int i = 0; i < Dim; i++)
					ret.max[i] = std::max(lhs.max[i], rhs.max[i]);
				return ret;
			}
		};

		template <typename _T, size_t _Align_Boundary = alignof(_T)>
		class aligned_allocator
		{
		public:
			typedef	_T		 value_type;
			typedef	size_t		 size_type;
			typedef	ptrdiff_t	 difference_type;
			typedef	_T		*pointer;
			typedef const _T		*const_pointer;
			typedef	_T		&reference;
			typedef const _T		&const_reference;
			inline aligned_allocator() throw() {}

			template <typename _T2>
			inline  aligned_allocator(const aligned_allocator<_T2, _Align_Boundary> &) throw() {}
			inline ~aligned_allocator() throw() {}

			template <size_t _Other_Alignment>
			inline bool operator== (const aligned_allocator<_T, _Other_Alignment> & rhs) const
			{
				return _Align_Boundary == _Other_Alignment;
			}

			inline pointer adress(reference r)
			{
				return &r;
			}

			inline const_pointer adress(const_reference r) const
			{
				return &r;
			}

			inline pointer allocate(size_type n)
			{
				return (pointer)_mm_malloc(n * sizeof(value_type), _Align_Boundary);
			}

			inline void deallocate(pointer p, size_type)
			{
				_mm_free(p);
			}

			inline void construct(pointer p, const_reference wert)
			{
				::new(p) value_type(wert);
			}

			inline void destroy(pointer p)
			{ /* C4100 */ p; p->~value_type();
			}

			inline size_type max_size() const throw()
			{
				return size_type(-1) / sizeof(value_type);
			}

			template <typename _T2>
			struct rebind { typedef aligned_allocator<_T2, _Align_Boundary> other; };
		};

		// aligned container
		template <typename _Ty>
		using aligned_vector = std::vector<_Ty, internal::aligned_allocator<_Ty>>;

	} // end namespace internal

	namespace concept
	{
		//concept 
		class concept_bvh
		{
			using node_handle = int;
			using aabb_t = int;
			using object_t = void*;
			template <typename _Ty>
			using range = _Ty*;

			node_handle			root() const;
			range<node_handle>	get_children(node_handle nid) const;
			const object_t*		get_objects(node_handle nid) const;
			aabb_t				get_volumn(node_handle nid) const;
		};
	}

	// K-dimensional Bounding Volumn Hierachy
	// naive implementation of a kdtree alike bvh structure
	// the bvh partition is done in axis 0,1,2,...,n blindly
	template <typename _TObject, typename _TScalar, size_t _Dim, typename _TBoundingVolumn = internal::aabb_t<_TScalar, _Dim>, typename _ContainerType = internal::aligned_vector<_TObject>>
	class kd_bvh
	{
	public:
		// Dummy Template Data
		//using _TObject = void*;
		//using _TScalar = float;
		//static constexpr size_t _Dim = 3;

		static constexpr size_t Dim = _Dim;
		using scalar_t = _TScalar;
		using object_t = _TObject;
		using volumn_t = _TBoundingVolumn;//internal::aabb_t<scalar_t, _Dim>;
		using idx_t = int;
		using node_handle = idx_t;
		using container_t = _ContainerType;
		static constexpr idx_t nil = -1;
		using volumn_func = ::std::function<volumn_t(const _TObject&)>;


	protected:
		using idxbox_t = struct volumn_idx_pair {
			volumn_t box;
			idx_t  index;
		};
		using idxbox_list = internal::aligned_vector<idxbox_t>;

		struct node_t
		{
			volumn_t box;
			idx_t	 parent;
			::std::pair<idx_t, idx_t>  children;
		};

		container_t					m_objects;
		internal::aligned_vector<node_t>		m_nodes;
		idx_t						m_root;
		volumn_func					mf_getbox;
		bool						m_valiad;

	public:
		kd_bvh() : m_valiad(false) {}
		kd_bvh(const volumn_func& get_volumn_func) : mf_getbox(get_volumn_func), m_valiad(false) {}
		kd_bvh(container_t && cont, const volumn_func& get_volumn_func) : m_objects(cont), mf_getbox(get_volumn_func), m_valiad(false) {
			rebuild();
		}

		node_handle get_parent(node_handle nid) const { return m_nodes[nid].parent; }

		iterator_range<const node_handle*> get_children(node_handle nid) const
		{
			auto& c = m_nodes[nid].children;
			// when nid is leaf, return empty range
			return ::std::make_range(&c.first, &c.first + (c.first != nil ? 2 : 0));
		}

		const volumn_t& get_volumn(node_handle nid) const
		{
			return m_nodes[nid].box;
		}

		const object_t* get_object(node_handle nid) const
		{
			return nid < m_objects.size() ? &m_objects[nid] : nullptr;
		}

		node_handle root() const { return m_root; }

		bool		valiad() const { return m_valiad; }

		inline auto size() const { return m_objects.size(); }
		inline bool empty() const { return m_objects.empty(); }
		// Const Accessor for the objects, random access range
		inline auto& operator[](size_t idx) const { return m_objects[idx]; }
		inline auto& at(size_t idx) const { return m_objects.at(idx); }
		inline auto begin() const { return m_objects.begin(); }
		inline auto end() const { return m_objects.end(); }
		inline auto rbegin() const { return m_objects.rbegin(); }
		inline auto rend() const { return m_objects.rend(); }
		inline auto& front() const { return m_objects.front(); }
		inline auto& back() const { return m_objects.back(); }

		// Mutable Modifiers & Accessors
		inline auto begin() { m_valiad = false; return m_objects.begin(); }
		inline auto end() { m_valiad = false; return m_objects.end(); }
		inline auto rbegin() { m_valiad = false; return m_objects.rbegin(); }
		inline auto rend() { m_valiad = false; return m_objects.rend(); }
		inline auto& front() { m_valiad = false;  return m_objects.front(); }
		inline auto& back() { m_valiad = false; return m_objects.back(); }
		inline auto& operator[](size_t idx) { m_valiad = false; return m_objects[idx]; }
		inline auto& at(size_t idx) { m_valiad = false; return m_objects.at(idx); }
		inline void resize(size_t newsize) { m_valiad = false;  m_objects.resize(newsize); }
		inline void clear() { m_valiad = false;  m_objects.clear(); }
		inline void push_back(const object_t &obj) { m_valiad = false;  m_objects.push_back(obj); }

		void  rebuild()
		{
			if (!m_objects.size()) return;
			idx_t n = static_cast<idx_t>(m_objects.size());
			idxbox_list idxboxes(m_objects.size());
			m_nodes.reserve(n * 2 - 1);
			m_nodes.resize(n);
			for (idx_t i = 0; i < n; i++)
			{
				idxboxes[i].index = i;
				idxboxes[i].box = mf_getbox(m_objects[i]);
				m_nodes[i].box = idxboxes[i].box;
				m_nodes[i].parent = nil;
				m_nodes[i].children.first = nil;
				m_nodes[i].children.second = nil;
			}
			m_root = build<0>(idxboxes, 0, n);
			m_valiad = true;
		}

	protected:
		node_handle insert(const object_t&)
		{
		}

		void remove(node_handle nid)
		{
		}
		node_handle insert(const node_t&, node_handle nid)
		{
		}

		template <idx_t dim>
		idx_t build(idxbox_list& idxboxes, idx_t begin, idx_t end)
		{
			if (begin >= end) return nil;
			auto idxboxes_b = idxboxes.begin();
			static constexpr idx_t nextdim = (dim + 1) % Dim;
			// comparetor for boxes in dimension dim 
			auto box_less = [](const idxbox_t& lhs, const idxbox_t& rhs) -> bool {
				return volumn_t::less<dim>(lhs.box, rhs.box);
			};
			idx_t this_idx;
			if (end - begin == 1)
			{
				this_idx = idxboxes[begin].index;
				auto& node = m_nodes[this_idx];
				node.children.first = nil;
				node.children.second = nil;
			}
			else
			{
				this_idx = static_cast<idx_t>(m_nodes.size());
				m_nodes.emplace_back();
				auto& node = m_nodes.back();

				idx_t mid = begin + (end - begin) / 2;
				if (end - begin > 2) // when l==2, no need to sort
					::std::nth_element(idxboxes_b + begin, idxboxes_b + mid, idxboxes_b + end, box_less);

				idx_t lc = build<nextdim>(idxboxes, begin, mid);
				idx_t rc = build<nextdim>(idxboxes, mid, end);
				node.children = std::make_pair(lc, rc);
				// box of this node is the merge of its children
				node.box = m_nodes[lc].box + m_nodes[rc].box;
				node.parent = nil;
				m_nodes[lc].parent = m_nodes[rc].parent = this_idx;
			}
			return this_idx;
		}
	};

	template <typename _TDerived, typename _Tkdtree, typename _TPred>
	class bvh_itrerator_base
		: public ::std::iterator<::std::input_iterator_tag, ::std::add_const_t<typename _Tkdtree::object_t>>
	{
	public:
		using tree_t = typename _Tkdtree;
		using node_handle = typename tree_t::node_handle;
		using volumn_t = typename tree_t::volumn_t;
		using object_t = typename tree_t::object_t;
		using pred_t = typename _TPred;

	protected:
		const tree_t*			_tree;
		pred_t					_pred;
		node_handle				_index;

	public:
		bvh_itrerator_base() : _index(tree_t::nil), _tree(nullptr) {}

		bvh_itrerator_base(const tree_t& tree, const pred_t& pred, node_handle start = tree_t::nil)
			: _pred(pred), _tree(&tree)
		{
			if (start == tree_t::nil)
				_index = _tree->root();
		}

		bool is_end() const { return this->_index == tree_t::nil; }
		const object_t& get() const { return *this->_tree->get_object(this->_index); }
		operator bool() const { return !this->is_end(); }
		const object_t& operator*() const { return this->get(); }

		bool operator++() { return static_cast<_TDerived*>(this)->advance(); }
		bool operator++(int) { return static_cast<_TDerived*>(this)->advance(); }

		bool operator != (const _TDerived& rhs) const { return this->_index != rhs._index; }
		template <typename _U>
		bool operator != (const _U&) const { return !this->is_end(); }
	};


	template <typename _Tkdtree, typename _TPred>
	class bvh_dfs_itrerator
		: public bvh_itrerator_base<bvh_dfs_itrerator<_Tkdtree, _TPred>, _Tkdtree, _TPred>
	{
	public:
		using base_type = bvh_itrerator_base<bvh_dfs_itrerator<_Tkdtree, _TPred>, _Tkdtree, _TPred>;
		std::stack<node_handle> _todo;

	public:
		bvh_dfs_itrerator() = default;

		bvh_dfs_itrerator(const tree_t& tree, const pred_t& pred, node_handle start = tree_t::nil)
			: base_type(tree, pred, start)
		{
			if (this->_pred(this->_tree->get_volumn(_index)))
				this->_todo.push(_index);
			advance();
		}

		bool advance() {
			if (this->_index == tree_t::nil) return false;
			while (!this->_todo.empty())
			{
				this->_index = this->_todo.top();
				this->_todo.pop();
				auto children = this->_tree->get_children(this->_index);

				for (auto& cnode : children)
					if (this->_pred(this->_tree->get_volumn(cnode)))
						this->_todo.push(cnode);

				auto obj = this->_tree->get_object(this->_index);
				if (obj != nullptr) return true;
			}

			this->_index = -1;
			return false;
		}
	};

	// A* search the bounding volumn herachy
	template <typename _Tkdtree, typename _TObjPred, typename _TVolumnPred>
	const typename _Tkdtree::object_t* find_first_of(const _Tkdtree& tree, _TObjPred&& distance_obj, _TVolumnPred&& lowerbound_vol)
	{
		using tree_t = _Tkdtree;
		using scalar_t = typename tree_t::scalar_t;
		using node_handle = typename tree_t::node_handle;
		using volumn_t = typename tree_t::volumn_t;
		using object_t = typename tree_t::object_t;

		struct queue_node
		{
			node_handle handle;
			scalar_t	lower_bound;
			// We want a min heap
			bool operator < (const queue_node& rhs) const {
				return this->lower_bound > rhs.lower_bound;
			}
		};

		std::priority_queue<queue_node> todo;
		node_handle						best_node = tree_t::nil;
		scalar_t						best_distance = std::numeric_limits<scalar_t>::max();

		queue_node qn{ tree.root(), lowerbound_vol(tree.get_volumn(tree.root())) };
		if (qn.lower_bound < 0) return nullptr; // not intersected at all
		else
			todo.push(qn);

		while (!todo.empty() && todo.top().lower_bound < best_distance)
		{
			qn = todo.top();
			node_handle node = qn.handle;
			todo.pop();
			//std::cout << "Testing node " << node << ", lower_bound = " << qn.lower_bound << std::endl;

			scalar_t dis;
			auto pobj = tree.get_object(node);
			if (pobj) // left node, check and update the actual distance
			{
				dis = distance_obj(*pobj);
				if (!(dis < 0) && dis < best_distance)
				{
					best_node = node;
					best_distance = dis;
					//std::cout << "Update best node " << node << ", best_distance = " << dis << std::endl;
				}
			}
			else // besides, add children to the queue
			{
				auto children = tree.get_children(node);
				for (auto& cnode : children)
				{
					dis = lowerbound_vol(tree.get_volumn(cnode));
					if (!(dis < 0) && dis < best_distance)
						todo.push(queue_node{ cnode, dis });
				}
			}
		}

		if (best_node != tree_t::nil)
			return tree.get_object(best_node);
		return nullptr;
	}

	template <typename _Tkdtree, typename _TPred>
	auto find_all_of(const _Tkdtree& bvh, const _TPred &pred)
	{
		return bvh_dfs_itrerator<_Tkdtree, _TPred>(bvh, pred);
	}
}