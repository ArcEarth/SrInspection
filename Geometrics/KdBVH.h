#pragma once
//#include "DirectXMathExtend.h"
#include <vector>
#include <algorithm>
#include <minmax>
#include <iterator>
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

		//internal pair class for the BVH--used instead of std::pair because of alignment
		template<typename _Scalar, size_t _Dim>
		struct box_int_pair_t : public aabb_t<_Scalar, _Dim>
		{
			using base_t = aabb_t<_Scalar, _Dim>;
			using box_t = base_t;
			int index;

			box_t& box() { return static_cast<box_t&>(*this); }
			const box_t& box() const { return static_cast<const box_t&>(*this); }
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
			range<object_t>		get_objects(node_handle nid) const;
			aabb_t				get_volumn(node_handle nid) const;
		};
	}

	template <typename _TObject, typename _TScalar, size_t _Dim>
	class kd_tree
	{
	public:
		// Dummy Template Data
		//using _TObject = void*;
		//using _TScalar = float;
		//static constexpr size_t _Dim = 3;

		static constexpr size_t Dim = _Dim;
		using scalar_t = _TScalar;
		using aabb_t = internal::aabb_t<scalar_t, _Dim>;
		using idxbox_t = internal::box_int_pair_t<scalar_t, _Dim>;
		using idx_t = int;
		using object_t = _TObject;
		static constexpr idx_t nil = -1;
		using volumn_func = ::std::function<aabb_t(const _TObject&)>;


	protected:
		// aligned container
		template <typename _Ty>
		using aligned_vector = std::vector<_Ty, internal::aligned_allocator<_Ty>>;

		using idxbox_list = aligned_vector<idxbox_t>;

		struct node_t
		{
			aabb_t box;
			::std::pair<idx_t, idx_t>  children;
		};
		::std::vector<_TObject>	m_objects;
		aligned_vector<node_t>	m_nodes;
		idx_t					m_root;
		volumn_func				mf_getbox;

	public:
		using node_handle = idx_t;

		kd_tree() = default;
		kd_tree(const volumn_func& get_volumn_func) :mf_getbox(get_volumn_func) {}
		template <typename _ObjIter>
		kd_tree(_ObjIter _begin, _ObjIter _end, const volumn_func& get_volumn_func)
			: m_objects(_begin, _end), mf_getbox(get_volumn_func)
		{
			rebuild();
		}

		iterator_range<const node_handle*> get_children(node_handle nid) const
		{
			auto& c = m_nodes[nid].children;
			// when nid is leaf, return empty range
			return ::std::make_range(&c.first, &c.first + (c.first != nil ? 2 : 0));
		}

		const aabb_t& get_volumn(node_handle nid) const
		{
			return m_nodes[nid].box;
		}

		const object_t* get_object(node_handle nid) const
		{
			return nid < m_objects.size() ? &m_objects[nid] : nullptr;
		}

		node_handle root() const { return m_root; }

		template <typename _ObjIter>
		void rebuild(_ObjIter _begin, _ObjIter _end, const volumn_func)
		{
			m_objects.assign<_ObjIter>(_begin, _end);
			rebuild();
		}

		void  rebuild()
		{
			idx_t n = static_cast<idx_t>(m_objects.size());
			idxbox_list idxboxes(m_objects.size());
			m_nodes.reserve(n * 2 - 1);
			m_nodes.resize(n);
			for (idx_t i = 0; i < n; i++)
			{
				idxboxes[i].index = i;
				idxboxes[i].box() = mf_getbox(m_objects[i]);
				m_nodes[i].box = idxboxes[i].box();
			}
			m_root = build<0>(idxboxes, 0, n);
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
				return lhs.min[dim] + lhs.max[dim] < rhs.min[dim] + rhs.max[dim];
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
			}
			return this_idx;
		}
	};

	template <typename _Tkdtree, typename _TPred>
	class kd_tree_pred_iterator
		: public ::std::iterator<::std::forward_iterator_tag, ::std::add_const_t<typename _Tkdtree::object_t>>
	{
		using this_type = kd_tree_pred_iterator<_Tkdtree, _TPred>;
		using tree_t = typename _Tkdtree;
		using node_handle = typename tree_t::node_handle;
		using aabb_t = typename tree_t::aabb_t;
		using object_t = typename tree_t::object_t;
		using pred_t = typename _TPred;

		const tree_t*			_tree;
		pred_t					_pred;
		std::stack<node_handle> _todo;
		node_handle				_index;
		const object_t*			_obj;

	public:
		kd_tree_pred_iterator() : _index(tree_t::nil), _tree(nullptr), _obj(nullptr) {}

		kd_tree_pred_iterator(const tree_t& tree, const pred_t& pred, node_handle start = tree_t::nil)
			: _pred(pred), _tree(&tree)
		{
			if (start == tree_t::nil)
				_index = _tree->root();
			_todo.push(_index);
			//_obj = _tree->get_object(_index);
			//if (!_obj) 
			advance();
		}

		bool is_end() const { return this->_index == tree_t::nil; }

		const object_t& get() const {
			return *this->_obj;
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

				this->_obj = this->_tree->get_object(this->_index);
				if (this->_obj != nullptr) return true;
			}

			this->_index = -1;
			return false;
		}

		operator bool() const { return !this->is_end(); }
		const object_t& operator*() const { return this->get(); }
		bool operator++() { return this->advance(); }
		bool operator++(int) { return this->advance(); }
		bool operator != (const this_type& rhs) const {
			return this->_index != rhs._index;
		}
		template <typename _U>
		bool operator != (const _U&) const { return !this->is_end(); }
	};

	template <typename _Tkdtree, typename _TPred>
	auto find_all_if(const _Tkdtree& bvh, const _TPred &pred)
	{
		return kd_tree_pred_iterator<_Tkdtree, _TPred>(bvh, pred);
	}
}