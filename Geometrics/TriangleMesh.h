#pragma once

#include <utility>
#include <array>
#include <vector>
#include <unordered_map>
#include <DirectXMathExtend.h>
#include <gsl.h>
#include <VertexTraits.h>
#include <minmax>
#include "bvh.h"
#include <hlslm\hlsl.hpp>
#include <iostream>

namespace Geometrics
{
	template <typename IndexType>
	struct Triangle
	{
		static constexpr size_t VertexCount = 3;
		typedef std::array<IndexType, VertexCount> container_type;
		container_type v;

		typedef IndexType value_type;
		typedef size_t size_type;
		typedef ptrdiff_t difference_type;
		typedef IndexType *pointer;
		typedef const IndexType *const_pointer;
		typedef IndexType& reference;
		typedef const IndexType& const_reference;

		typedef typename container_type::iterator iterator;
		typedef typename container_type::const_iterator const_iterator;

		typedef typename container_type::reverse_iterator reverse_iterator;
		typedef typename container_type::const_reverse_iterator const_reverse_iterator;

		Triangle() {}
		Triangle(const IndexType &v0, const IndexType &v1, const IndexType &v2)
		{
			v[0] = v0; v[1] = v1; v[2] = v2;
		}
		Triangle(const IndexType *v_)
		{
			v[0] = v_[0]; v[1] = v_[1]; v[2] = v_[2];
		}
		template <class S> explicit Triangle(const S &x)
		{
			v[0] = x[0];  v[1] = x[1];  v[2] = x[2];
		}
		IndexType &operator[] (int i) { return v[i]; }
		const IndexType &operator[] (int i) const { return v[i]; }
		operator const IndexType * () const { return &(v[0]); }
		operator IndexType * () { return &(v[0]); }

		void rewind() {
			std::swap(v[0], v[2]);
		}

		int indexof(IndexType v_) const
		{
			return (v[0] == v_) ? 0 :
				(v[1] == v_) ? 1 :
				(v[2] == v_) ? 2 : -1;
		}

		// contrainer access
		auto begin() { return v.begin(); }
		auto end() { return v.end(); }
		auto begin() const { return v.begin(); }
		auto end() const { return v.end(); }

		auto rbegin() { return v.rbegin(); }
		auto rend() { return v.rend(); }
		auto rbegin() const { return v.rbegin(); }
		auto rend() const { return v.rend(); }

		size_t size() const { return VertexCount; }
	};

	struct submesh_data
	{
		uint32_t vertex_offset; // start vertices index of this geometry
		uint32_t vertex_count;
		uint32_t index_offset; // start index position of this geometry
		uint32_t index_count;
	};


	// generate per vertex normal for given triangle mesh
	template <typename VertexType, typename IndexType>
	bool generate_normal(gsl::span<VertexType> vertices, gsl::span<const Triangle<IndexType>> facets)
	{
		using namespace DirectX::VertexTraits;
		using namespace DirectX;

		if (!has_normal<VertexType>::value)
			return false;

		//static_assert(has_normal<VertexType>::value, "The vertex type dose not contains normal field");

		std::vector<XMVECTOR, XMAllocator> normals(vertices.size());

		// set zero
		std::memset(normals.data(), 0, normals.size() * sizeof(XMVECTOR));

		XMVECTOR v0, v1, v2, n;
		for (const auto& face : facets)
		{
			v0 = get_position(vertices[face[0]]);
			v1 = get_position(vertices[face[1]]);
			v2 = get_position(vertices[face[2]]);
			v1 -= v0;
			v2 -= v0;

			v1 = XMVector3Normalize(v1);
			v2 = XMVector3Normalize(v2);
			n = XMVector3Cross(v1, v2); // weighted normal 

			normals[face[0]] += n;
			normals[face[1]] += n;
			normals[face[2]] += n;
		}

		for (size_t i = 0; i < vertices.size(); i++)
		{
			n = XMVector3Normalize(normals[i]);
			set_normal(vertices[i], n);
		}

		return true;
	}

	// generate per vertex tangent for given triangle mesh
	template <typename VertexType, typename IndexType>
	bool generate_tangent(gsl::span<VertexType> vertices, gsl::span<const Triangle<IndexType>> facets)
	{
		using namespace DirectX::VertexTraits;
		using namespace DirectX;

		if (!has_tangent<VertexType>::value)
			return false;
		//static_assert(has_tangent<VertexType>::value, "The vertex type dose not contains tangent field");

		std::vector<XMVECTOR, XMAllocator> tan1(vertices.size() * 2);
		XMVECTOR* tan2 = &tan1[vertices.size()];
		std::memset(tan1.data(), 0, tan1.size() * sizeof(XMVECTOR));

		for (const auto& face : facets)
		{
			{
				XMVECTOR v0, v1, v2, w0, w1, w2;
				v0 = get_position(vertices[face[0]]);
				w0 = get_uv(vertices[face[0]]);

				v1 = get_position(vertices[face[1]]);
				w1 = get_uv(vertices[face[1]]);

				v2 = get_position(vertices[face[2]]);
				w2 = get_uv(vertices[face[2]]);
			}
			XMFLOAT4A v_1, v_2, w_1, w_2;

			XMStoreA(v_1, v1 - v0);
			XMStoreA(v_2, v2 - v0);
			XMStoreA(w_1, w1 - w0);
			XMStoreA(w_2, w2 - w0);

			float x1 = v_1.x; float x2 = v_2.x;
			float y1 = v_1.y; float y2 = v_2.y;
			float z1 = v_1.z; float z2 = v_2.z;

			float s1 = w_1.x; float s2 = w_2.x;
			float t1 = w_1.y; float t2 = w_2.y;

			float r = 1.0F / (s1 * t2 - s2 * t1);
			XMVECTOR sdir = XMVectorSet(
				(t2 * x1 - t1 * x2) * r,
				(t2 * y1 - t1 * y2) * r,
				(t2 * z1 - t1 * z2) * r, .0f);
			XMVECTOR tdir = XMVectorSet(
				(s1 * x2 - s2 * x1) * r,
				(s1 * y2 - s2 * y1) * r,
				(s1 * z2 - s2 * z1) * r, .0f);

			tan1[face[0]] += sdir;
			tan1[face[1]] += sdir;
			tan1[face[2]] += sdir;

			tan2[face[0]] += tdir;
			tan2[face[1]] += tdir;
			tan2[face[2]] += tdir;
		}

		for (size_t i = 0; i < vertices.size(); i++)
		{
			XMVECTOR n = get_normal(vertices[i], n);
			XMVECTOR t = tan1[i];

			// Gram-Schmidt orthogonalize
			XMVECTOR nt = XMVector3Normalize(t - n * XMVector3Dot(n, t));

			XMVECTOR w = XMVectorLess(XMVector3Dot(XMVector3Cross(n, t), tan2[i]), XMVectorZero());
			w = XMVectorSelect(g_XMNegativeOne.v, g_XMOne.v, w);

			nt = _DXMEXT XMVectorSelect<0, 0, 0, 1>(nt, w);
			set_tangent(vertices[i], nt);
		}

		return true;
	}

	namespace Detail
	{
		XM_HAS_MEMBER(vertices, has_vertices);
		XM_HAS_MEMBER(indices, has_indices);
	}

	template <typename MeshType>
	struct concept_mesh_type : public std::integral_constant<bool, Detail::has_vertices<MeshType>::value && Detail::has_indices<MeshType>::value> {};

	template <typename _MeshType>
	class SubMesh : public submesh_data
	{
	public:
		typedef _MeshType MeshType; // Parent MeshTypw
		typedef typename MeshType::VertexType	VertexType;
		typedef typename MeshType::IndexType	IndexType;
		typedef typename MeshType::FaceType		FaceType;

		static constexpr size_t PolygonVertex = FaceType::VertexCount;


		SubMesh(MeshType& mesh, const submesh_data& metric)
			: parent(mesh), submesh_data(metric)
		{
			vertices = gsl::span<VertexType>(&mesh.vertices[metric.vertex_offset], metric.vertex_count);
			indices = gsl::span<IndexType>(&mesh.indices[metric.index_offset], metric.index_count);
		}

		MeshType&						parent;
		gsl::span<VertexType>			vertices;
		gsl::span<IndexType>			indices;

		gsl::span<Triangle<IndexType>>	facets()
		{
			return gsl::span<const FaceType>(
				reinterpret_cast<const FaceType*>(indices.data()),
				indices.size() / FaceType::VertexCount);
		}

		bool empty() const {
			return vertices.empty() || indices.empty();
		}

	};

	template <typename _VertexType, typename _IndexType = uint16_t, typename _FaceType = Triangle<_IndexType>>
	class PolygonSoup
	{
	public:
		typedef _VertexType VertexType;
		typedef _IndexType	IndexType;
		typedef _FaceType	FaceType;

		static constexpr size_t PolygonVertex = FaceType::VertexCount;

		PolygonSoup() {}

		bool empty() const {
			return vertices.empty() || indices.empty();
		}

		void clear()
		{
			vertices.clear();
			indices.clear();
		}

		inline const FaceType& facet(int idx) const
		{
			return reinterpret_cast<const FaceType&>(indices[idx * FaceType::VertexCount]);
		}

		inline FaceType& facet(int idx)
		{
			return reinterpret_cast<FaceType&>(indices[idx * FaceType::VertexCount]);
		}

		void add_facet(const FaceType& new_facet)
		{
			indices.insert(indices.end(), new_facet.begin(), new_facet.end());
		}

		template <class... _TIndecies>
		std::enable_if_t<sizeof...(_TIndecies) == FaceType::VertexCount> add_facet(_TIndecies... _indices)
		{
			indices.insert(indices.end(), { static_cast<IndexType>(_indices)... });
		}


		gsl::span<FaceType> facets() {
			return gsl::span<FaceType>(
				reinterpret_cast<FaceType*>(indices.data()),
				indices.size() / FaceType::VertexCount);
		}

		gsl::span<const FaceType> facets() const {
			return gsl::span<const FaceType>(
				reinterpret_cast<const FaceType*>(indices.data()),
				indices.size() / FaceType::VertexCount);
		}

		const VertexType& vertex(int facet, int vidx) const
		{
			return this->vertices[this->indices[facet * FaceType::VertexCount + vidx]];
		}

		VertexType& vertex(int facet, int vidx)
		{
			return this->vertices[this->indices[facet * FaceType::VertexCount + vidx]];
		}

		// flip all the polygons' winding and vertices' normal 
		void flip()
		{
			using namespace DirectX::VertexTraits;

			if (has_normal<VertexType>::value)
			{
				for (auto& v : vertices)
					set_normal(v, -get_normal(v));
			}

			static constexpr int vc = FaceType::VertexCount;
			for (int i = 0; i < indices.size() / vc; i++)
				std::reverse(indices.begin() + i * vc, indices.begin() + (i + 1)* vc);
		}

		void generate_normal()
		{
			static_assert(PolygonVertex == 3, "This mesh is not trigle mesh, please trianglize it first");
			generate_normal<VertexType, IndexType>(this->vertices, this->facets());
		}

		void generate_tangent()
		{
			static_assert(PolygonVertex == 3, "This mesh is not trigle mesh, please trianglize it first");
			generate_tangent<VertexType, IndexType>(this->vertices, this->facets());
		}

		// applies an uniform transform to all vertices in the mesh 
		void XM_CALLCONV transform(DirectX::FXMMATRIX M)
		{
			using namespace DirectX::VertexTraits;
			for (auto& v : vertices)
			{
				XMVECTOR p = get_position(v);
				p = _DXMEXT XMVector3TransformCoord(p, M);
				set_position(v, p);

				if (has_normal<VertexType>::value)
				{
					p = get_normal(v);
					p = _DXMEXT XMVector3TransformNormal(p, M);
					set_normal(v, p);

					if (has_tangent<VertexType>::value)
					{
						p = get_tangent(v);
						p = _DXMEXT XMVector3TransformNormal(p, M);
						set_tangent(v, p);
					}
				}
			}
		}

		std::vector<VertexType, DirectX::AlignedAllocator<VertexType>> vertices;
		std::vector<IndexType>	indices;
	};

	template <class _VertexType, class _IndexType>
	int XM_CALLCONV intersect(const PolygonSoup<_VertexType, _IndexType, Triangle<_IndexType>> &Mesh, DirectX::FXMVECTOR Origin, DirectX::FXMVECTOR Direction, std::vector<float>* distances = nullptr)
	{
		using namespace DirectX;
		using namespace DirectX::VertexTraits;
		int count = 0;
		XMVECTOR vDir = XMVector3Normalize(Direction);
		for (const auto& tri : Mesh.facets())
		{
			float distance;

			XMVECTOR v0 = get_position(&Mesh.vertices[tri[0]]);
			XMVECTOR v1 = get_position(&Mesh.vertices[tri[1]]);
			XMVECTOR v2 = get_position(&Mesh.vertices[tri[2]]);

			bool hr = DirectX::TriangleTests::Intersects(Origin, vDir, v0, v1, v2, distance);
			if (hr) {
				++count;
				if (distances) {
					distances->push_back(distances);
				}
			}
		}
		return count;
	}

	template <typename _TVertex>
	struct MeshRayIntersectedVertex : public _TVertex
	{
		using Vector3 = DirectX::Vector3;

		float	distance;
		Vector3 barycentric;
		int		facet; // facet id

		inline bool operator < (const MeshRayIntersectedVertex& rhs)
		{
			return this->distance < rhs.distance;
		}

		inline bool is_valiad() const {return  facet >= 0; }
		inline operator bool() const { return is_valiad(); }
	};

	template <size_t _Dim>
	struct PolygonContainmentInfo
	{
		static constexpr size_t Dim = _Dim;
		DirectX::ContainmentType containment; // overall polygon containment type
		DirectX::ContainmentType vertex_containments[Dim];
		operator DirectX::ContainmentType() const { return containment; }
	};

	struct XM_ALIGNATTR aabb_t {
		::hlsl::xmvector3f min;
		::hlsl::xmvector3f max;
		mutable int	 intersected;

		template <size_t dim>
		static bool less(const aabb_t& lhs, const aabb_t & rhs)
		{
			using namespace hlsl;
			return hlsl::less(lhs.min, rhs.min).get<dim>();
		}

		aabb_t operator+(const aabb_t& rhs) const
		{
			const aabb_t& lhs = *this;
			using namespace ::hlsl;
			aabb_t ret{ ::hlsl::min(lhs.min,rhs.min),::hlsl::max(lhs.max,rhs.max) };
			return ret;
		}

		// Convert min-max representation of this box to center-extent representation
		DirectX::BoundingBox get_dxbox() const
		{
			const float epsilon = 1e-3;
			using namespace ::hlsl;
			auto& aabb = *this;
			DirectX::BoundingBox box;
			xmvector3f extent = (aabb.max - aabb.min) * 0.5f;
			extent = ::hlsl::max(extent, xmfloat(epsilon));
			box.Center << (aabb.max + aabb.min) * 0.5f;
			box.Extents << extent;
			return box;
		}

		operator DirectX::BoundingBox() const { return get_dxbox(); }
	};

	// A struct stores fixed size polygon (position only)
	template <size_t _Size>
	struct PlainPolygon {
		hlsl::xmvector3f v[_Size];
	};

	/// <summary>
	/// Basic triangle mesh, each index represent an edge, which is the edge oppsite to the vertex in it's owner triangle
	/// </summary>
	template <typename _VertexType, typename _IndexType = uint16_t>
	class TriangleMesh : public PolygonSoup<_VertexType, _IndexType, Triangle<_IndexType>>
	{
	public:
		static const size_t VertexCount = FaceType::VertexCount;
		using triangle_handle = IndexType;
		using PlainTriangle = PlainPolygon<3>;
		using ContainmentType = DirectX::ContainmentType;
		using BaseType = PolygonSoup<_VertexType, _IndexType, Triangle<_IndexType>>;
		using BaseType::PolygonVertex;
		using typename BaseType::VertexType;
		using typename BaseType::IndexType;
		using IntersectionVertex = MeshRayIntersectedVertex<_VertexType>;

	public:
		// edge's reverse edge
		// stores the adjacent edges of a edge in a triangle
		std::vector<IndexType>	revedges;
		DirectX::BoundingBox	aabb;
		DirectX::BoundingOrientedBox obb;

		PlainTriangle XM_CALLCONV get_triangle_position(IndexType fid) const
		{
			using namespace DirectX::VertexTraits;
			using namespace hlsl;
			auto f = this->indices.begin() + fid * 3;
			PlainTriangle tri = {
			xmvector3f(get_position(this->vertices[f[0]]))
			,xmvector3f(get_position(this->vertices[f[1]]))
			,xmvector3f(get_position(this->vertices[f[2]])) };
			return tri;
		}

#define _EXPANDTRI(tri) tri.v[0].v, tri.v[1].v, tri.v[2].v

		aabb_t XM_CALLCONV get_triangle_aabb(triangle_handle t) const
		{
			using namespace hlsl;
			PlainTriangle tri = get_triangle_position(t);
			xmvector3f vmax = hlsl::max(tri.v[0], tri.v[1]);
			vmax = hlsl::max(vmax, tri.v[2]);
			xmvector3f vmin = hlsl::min(tri.v[0], tri.v[1]);
			vmin = hlsl::min(vmin, tri.v[2]);
			aabb_t ret;
			ret.min = vmin;
			ret.max = vmax;
			ret.intersected = true;
			return ret;
		}

		using triangle_bvh_t = kd_bvh<triangle_handle, float, 3, aabb_t>;
		triangle_bvh_t				triangle_bvh;

		using vertex_bvh_t = kd_bvh<::hlsl::xmvector3f, float, 3, aabb_t>;
		vertex_bvh_t				vertex_bvh;
		// An Lookup table to find the 'Ienditification' of a vertex
		// Make sense when multiple vertex shares the same spatial location but have different index
		std::vector<IndexType>		vertex_id;

		TriangleMesh() : triangle_bvh([this](const triangle_handle& t) {
			return this->get_triangle_aabb(t);
		}) {}

		// vertex's first adjacant edge
		// std::vector<IndexType> vedges;

		union EdgeType
		{
			_IndexType v[2];
			struct {
				_IndexType v0, v1;
			};
		};

		inline EdgeType edge(int facet, int edge) const
		{
			EdgeType e;
			auto& tri = this->facet(facet);
			switch (edge)
			{
			case 0:
				e.v0 = tri[1];
				e.v1 = tri[2];
				break;
			case 1:
				e.v0 = tri[2];
				e.v1 = tri[0];
				break;
			case 2:
				e.v0 = tri[0];
				e.v1 = tri[1];
				break;
			default:
				e.v0 = -1;
				e.v1 = -1;
				break;
			}
			return e;
		}

		inline EdgeType edge(int eid) const
		{
			return edge(eid / VertexCount, eid % VertexCount);
		}

		inline int adjacentFacet(int facet, int edge) const
		{
			return revedges[facet * VertexCount + edge] / VertexCount;
		}

		inline int adjacentFacet(int eid) const
		{
			return revedges[eid] / VertexCount;
		}

		void build_reverse_edges()
		{
			// intialize all adjacant edges to -1
			revedges.resize(this->indices.size(), -1);
			_IndexType vsize = this->vertices.size();
			int esize = this->indices.size();
			int fsize = this->indices.size() / VertexCount;

			using hash_type = uint64_t;
			// make sure int for hash is enough
			assert(vsize*vsize < std::numeric_limits<hash_type>::max());
			std::unordered_map<hash_type, _IndexType> edges(esize * 2);
			// max items inside this table should be less than (esize / 2)

			for (_IndexType fid = 0; fid < fsize; fid++)
			{
				for (_IndexType i = 0; i < VertexCount; i++)
				{
					_IndexType eid = i + fid * VertexCount;
					auto e = this->edge(fid, i);
					e.v0 = vertex_id[e.v0];
					e.v1 = vertex_id[e.v1];

					hash_type ehash = e.v0 * vsize + e.v1;
					hash_type revehash = e.v0 + e.v1 * vsize;

					auto revItr = edges.find(revehash);
					if (revItr == edges.end())
						edges[ehash] = eid;
					else // find reversed edge, remove from edges map
					{
						auto revEid = revItr->second;
						revedges[eid] = revEid;
						revedges[revEid] = eid;
						auto reve = this->edge(revEid);
						reve.v0 = vertex_id[reve.v0];
						reve.v1 = vertex_id[reve.v1];
						assert(reve.v0 == e.v1 && reve.v1 == e.v0);
						edges.erase(revItr);
					}
				}
			}
		}

		// build the adjacent map so we can access all the 1 rings
		void build()
		{
			int vsize = this->vertices.size();
			int esize = this->indices.size();
			int fsize = this->indices.size() / VertexCount;

			// rebuild the triangle-bvh
			triangle_bvh.resize(fsize);
			std::iota(triangle_bvh.begin(), triangle_bvh.end(), 0);
			triangle_bvh.rebuild();

			// build the vertex-bvh
			const aabb_t& bb = this->triangle_bvh.get_volumn(this->triangle_bvh.root());
			::hlsl::xmvector3f voxel_ext(1e-4f);

			vertex_bvh.set_volumn_getter([voxel_ext](const ::hlsl::xmvector3f v) {
				return aabb_t{ v - voxel_ext , v + voxel_ext, 0 };
			});
			vertex_bvh.resize(vsize);
			for (int i = 0; i < vsize; i++)
				vertex_bvh[i] = ::hlsl::xmvector3f(::DirectX::VertexTraits::get_position(this->vertices[i]));
			vertex_bvh.rebuild();

			vertex_id.resize(vsize);
			std::iota(vertex_id.begin(), vertex_id.end(), 0);
			std::unordered_map<uint64_t, IndexType> vids(vsize);

			// Use 20-bits to hash a float component, plus 1 bit signbit
			// Generate a total 63-bits
			auto hash_position = [](const ::hlsl::xmvector3f vp) -> uint64_t{
				auto vf = vp * 3e4f; // 30 * 3e4 = 9e5 < 1e6 = 2^20
				auto vi = vf.cast<uint32_t>();
				uint32_t signbits32 = ::hlsl::detail::move_mask(vi);
				uint64_t signbits = signbits32;
				vi &= ::hlsl::xmuint((1ULL << 20ULL) - 1ULL);
				uint32_t vis32[3];
				vis32 << vi;
				uint64_t vis[3] = { vis32[0], vis32[1], vis32[2]};
				uint64_t vhash = ((signbits&7) << 60ULL) | (vis[0] << 40ULL) | (vis[1] << 20ULL) | (vis[2]);
				return vhash;
			};

			for (int i = 0; i < vsize; i++)
			{
				auto vp = *vertex_bvh.get_object(i);
				std::pair<uint64_t, IndexType> p = { hash_position(vp), i };
				auto rib = vids.insert(p);
				IndexType vid = rib.first->second;
				if (!rib.second && vid != i)
				{
					// query-merge-set merge algorithm
					int root = vid;
					// find the ultimate parent (root)
					while (vertex_id[root] != root) root = vertex_id[root];

					while (vid != root)
					{
						int cvid = vid;
						vid = vertex_id[vid];
						vertex_id[cvid] = root;
					}
					vertex_id[i] = root;
					//if (vid != i)
					//	std::cout << "Glued vertex pair (" << i << ',' << vid << ')' << std::endl;
				}
			}

			build_reverse_edges();


			//for (int fid = 0; fid < this->indices.size() / 3; ++fid)
			//{
			//	auto tri = this->facet(fid);
			//	using namespace DirectX::VertexTraits;
			//	DirectX::XMVECTOR v0 = get_position(this->vertices[tri[0]]);
			//	DirectX::XMVECTOR v1 = get_position(this->vertices[tri[1]]);
			//	DirectX::XMVECTOR v2 = get_position(this->vertices[tri[2]]);
			//	auto&& vol = triangle_bvh.get_volumn(fid);
			//	bool cond = true;
			//	assert(DirectX::XMVector3LessOrEqual(v0, vol.max.v));
			//	assert(DirectX::XMVector3LessOrEqual(v1, vol.max.v));
			//	assert(DirectX::XMVector3LessOrEqual(v2, vol.max.v));
			//	assert(DirectX::XMVector3LessOrEqual(vol.min.v, v0));
			//	assert(DirectX::XMVector3LessOrEqual(vol.min.v, v1));
			//	assert(DirectX::XMVector3LessOrEqual(vol.min.v, v2));
			//}
		}

		// generate a persoude vertex from the interpolation of the trianlge
		VertexType persudo_vertex(int fid, DirectX::FXMVECTOR baycentric) const
		{
			const auto& tri = this->facet(fid);

			const VertexType& v0 = this->vertices[tri[0]];
			const VertexType& v1 = this->vertices[tri[1]];
			const VertexType& v2 = this->vertices[tri[2]];

			using namespace DirectX::VertexTraits;
			return interpolate_vertex(baycentric, v0, v1, v2);
		}

		protected:
			IntersectionVertex XM_CALLCONV get_ray_intersected_vertex(DirectX::FXMVECTOR vOri, DirectX::FXMVECTOR vDir, IndexType fid) const
			{
				using DirectX::XMVECTOR;
				IntersectionVertex info;
				float distance = .0f;
				info.facet = -1;
				PlainTriangle tri = this->get_triangle_position(fid);
				bool hr = DirectX::TriangleTests::Intersects(vOri, vDir, _EXPANDTRI(tri), distance);
				if (!hr) return info; // No intersection, return with info.facet == -1
				// Fill the persudo-vertex
				XMVECTOR pos = distance * vDir + vOri;
				XMVECTOR baryc = DirectX::TriangleTests::BarycentricCoordinate(pos, _EXPANDTRI(tri));
				info.barycentric = baryc;
				info.facet = fid;
				info.distance = distance;
				(VertexType&)info = this->persudo_vertex(fid, baryc);
				return info;
			}

			template <typename _Ty>
			inline static void push_back_output_itr(nullptr_t itr, _Ty&& element)
			{
			}

			template <typename _OutItr, typename _Ty>
			inline static std::enable_if_t<!std::is_same<_OutItr,nullptr_t>::value> 
				push_back_output_itr(_OutItr& itr, _Ty&& element)
			{
				static_assert(std::is_same<std::decay_t<decltype(*itr)>,std::decay_t<_Ty>>::value,"Output Iterator's Type is not expected");
				*(itr++) = element;
			}

		public:

		IntersectionVertex XM_CALLCONV first_intersect(DirectX::FXMVECTOR Origin, DirectX::FXMVECTOR Direction) const
		{
			using namespace DirectX;
			assert(this->triangle_bvh.valiad() && "This triangle mesh is modified but not rebuild");

			XMVECTOR vOri = Origin;
			XMVECTOR vDir = XMVector3Normalize(Direction);
			auto ray_aabb_inter = [vOri, vDir](const aabb_t& aabb) -> float
			{
				float f;
				auto box = aabb.get_dxbox();
				bool succ = box.Intersects(vOri, vDir, f);
				aabb.intersected = succ;
				return succ ? f : -1.0f;
			};
			auto ray_triangle_inter = [this, vOri, vDir](const int fid) -> float
			{
				PlainTriangle tri = this->get_triangle_position(fid);
				float f;
				bool succ = DirectX::TriangleTests::Intersects(vOri, vDir, _EXPANDTRI(tri), f);
				return succ ? f : -1.0f;
			};

			//std::cout << counter << std::endl;
			auto pfid = find_first_of(this->triangle_bvh, ray_triangle_inter, ray_aabb_inter);
			IntersectionVertex info;
			info.facet = -1;
			if (pfid)
				info = get_ray_intersected_vertex(vOri, vDir, *pfid);
			return info;
		}

		// Ray intersection test with advanced infomation
		template <typename _TOutItr>
		int XM_CALLCONV all_intersections(DirectX::FXMVECTOR Origin, DirectX::FXMVECTOR Direction, _TOutItr outItr) const
		{
			using namespace DirectX;
			assert(this->triangle_bvh.valiad() && "This triangle mesh is modified but not rebuild");

			size_t count = 0;
			XMVECTOR vOri = Origin;
			XMVECTOR vDir = XMVector3Normalize(Direction);

			struct XM_ALIGNATTR ray_aabb_intersector {
				DirectX::XMVECTOR origin;
				DirectX::XMVECTOR direction;
				bool XM_CALLCONV operator ()(const aabb_t& aabb) const
				{
					float f;
					auto box = aabb.get_dxbox();
					bool succ = box.Intersects(origin, direction, f);
					aabb.intersected = succ;
					return succ;
				}
			} intersector{ vOri , vDir };

			for (auto triangles = find_all_of(triangle_bvh, intersector);
				triangles;
				++triangles)
			{
				int fid = *triangles;
				IntersectionVertex info = get_ray_intersected_vertex(vOri, vDir, fid);
				if (info.is_valiad()) {
					++count;
					push_back_output_itr(outItr, std::move(info));
				}
			}
			return count;
		}

		template <typename _TPred, typename _TContainer>
		void find_adjacant_facets_of(IndexType facet_id, _TPred&& pred, _TContainer& result_map) const
		{
			auto containment = pred(facet_id);
			result_map.insert(std::make_pair(facet_id, containment));
			if (!containment) return;
			for (int e = 0; e < PolygonVertex; e++)
			{
				int eid = facet_id * PolygonVertex + e;
				auto rev_edge = this->revedges[eid];
				auto adj_face = rev_edge / PolygonVertex;
				// if not visited yet
				if (rev_edge != -1 && result_map.find(adj_face) == result_map.end())
					this->find_adjacant_facets_of(adj_face, pred, result_map);
			}
		}


	};

	namespace Internal
	{
#ifndef min
		template <typename T>
		inline T clamp(T value, T minV, T maxV)
		{
			return std::max(std::min(value, maxV), minV);
		}
#else
		template <typename T>
		inline T clamp(T value, T minV, T maxV)
		{
			return max(min(value, maxV), minV);
		}
#endif
	}

	/// <summary>
	/// Compute the closest projection point from P0 to triangle(V0,V1,V2).
	/// </summary>
	/// <param name="P0">The p0.</param>
	/// <param name="V0">The v0.</param>
	/// <param name="V1">The v1.</param>
	/// <param name="V2">The v2.</param>
	/// <returns></returns>
	inline DirectX::XMVECTOR XM_CALLCONV Projection(DirectX::FXMVECTOR P0, DirectX::FXMVECTOR V0, DirectX::FXMVECTOR V1, DirectX::GXMVECTOR V2)
	{
		using namespace DirectX;
		using namespace Geometrics::Internal;
		XMVECTOR edge0 = V1 - V0;
		XMVECTOR edge1 = V2 - V0;
		XMVECTOR p0 = V0 - P0;

		//XMVectorBaryCentric();

		float a = XMVectorGetX(_DXMEXT XMVector3LengthSq(edge0));
		float b = XMVectorGetX(_DXMEXT XMVector3Dot(edge0, edge1));
		float c = XMVectorGetX(_DXMEXT XMVector3LengthSq(edge1));
		float d = XMVectorGetX(_DXMEXT XMVector3Dot(edge0, p0));
		float e = XMVectorGetX(_DXMEXT XMVector3Dot(edge1, p0));

		float det = a*c - b*b;
		float s = b*e - c*d;
		float t = b*d - a*e;

		if (s + t < det)
		{
			if (s < 0.f)
			{
				if (t < 0.f)
				{
					if (d < 0.f)
					{
						s = clamp(-d / a, 0.f, 1.f);
						t = 0.f;
					}
					else
					{
						s = 0.f;
						t = clamp(-e / c, 0.f, 1.f);
					}
				}
				else
				{
					s = 0.f;
					t = clamp(-e / c, 0.f, 1.f);
				}
			}
			else if (t < 0.f)
			{
				s = clamp(-d / a, 0.f, 1.f);
				t = 0.f;
			}
			else
			{
				float invDet = 1.f / det;
				s *= invDet;
				t *= invDet;
			}
		}
		else
		{
			if (s < 0.f)
			{
				float tmp0 = b + d;
				float tmp1 = c + e;
				if (tmp1 > tmp0)
				{
					float numer = tmp1 - tmp0;
					float denom = a - 2 * b + c;
					s = clamp(numer / denom, 0.f, 1.f);
					t = 1 - s;
				}
				else
				{
					t = clamp(-e / c, 0.f, 1.f);
					s = 0.f;
				}
			}
			else if (t < 0.f)
			{
				if (a + d > b + e)
				{
					float numer = c + e - b - d;
					float denom = a - 2 * b + c;
					s = clamp(numer / denom, 0.f, 1.f);
					t = 1 - s;
				}
				else
				{
					s = clamp(-e / c, 0.f, 1.f);
					t = 0.f;
				}
			}
			else
			{
				float numer = c + e - b - d;
				float denom = a - 2 * b + c;
				s = clamp(numer / denom, 0.f, 1.f);
				t = 1.f - s;
			}
		}

		return V0 + s * edge0 + t * edge1;
	}

	inline float XM_CALLCONV Distance(DirectX::FXMVECTOR P0, DirectX::FXMVECTOR V0, DirectX::FXMVECTOR V1, DirectX::GXMVECTOR V2)
	{
		using namespace DirectX;
		XMVECTOR vProj = Projection(P0, V0, V1, V2);
		vProj -= P0;
		return XMVectorGetX(_DXMEXT XMVector3Length(vProj));
	}

	template <typename _VertexType, typename _IndexType>
	inline float XM_CALLCONV Distance(const TriangleMesh<_VertexType, _IndexType> &Mesh, DirectX::FXMVECTOR Point)
	{
		using namespace DirectX;
		float minDis = numeric_limits<float>::max();
		for (const auto& tri : Mesh.facets())
		{
			float dis;

			XMVECTOR v0 = XMLoadFloat3(&Mesh.vertices[tri[0]].position);
			XMVECTOR v1 = XMLoadFloat3(&Mesh.vertices[tri[1]].position);
			XMVECTOR v2 = XMLoadFloat3(&Mesh.vertices[tri[2]].position);

			dis = Distance(Point, v0, v1, v2);
			if (dis < minDis) minDis = dis;
			//minDis = std::min(dis,minDis);
		}

		//XMVECTOR vDis = XMVectorReplicate(numeric_limits<float>::max());
		//for (const auto& vertex : Mesh.vertices)
		//{
		//	XMVECTOR vPos = XMLoadFloat3(&vertex.position);
		//	vPos -= Point;
		//	XMVECTOR dis = XMVector3Length(vPos);
		//	vDis = XMVectorMin(vDis,dis);
		//}

		//float minDisV = XMVectorGetX(vDis);
		//assert(minDisV >= minDis);
		return minDis;
		//return XMVectorGetX(minDis);
	}


	template <typename _VertexType, typename _IndexType>
	bool XM_CALLCONV Inside(const TriangleMesh<_VertexType, _IndexType> &Mesh, DirectX::FXMVECTOR Point)
	{
		XMFLOAT3A Direction;
		Direction.x = (float)std::rand() / (RAND_MAX + 1);
		Direction.y = (float)std::rand() / (RAND_MAX + 1);
		Direction.z = (float)std::rand() / (RAND_MAX + 1);
		XMVECTOR vDir = XMLoadFloat3A(&Direction);
		auto count = intersect(Mesh, Point, vDir, nullptr);
		return count & 1; //count % 2
	}



}