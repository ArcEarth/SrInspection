#pragma once

#include <vector>
#include <memory>
#include "DirectXMathExtend.h"
#include <span.h>

namespace DirectX {
	namespace LineSegmentTest
	{
		enum LineSegmentIntersectionType
		{
			Disjoint = 0,
			Paredlle = 1,
			Intersect = 2,
			Overlap = 3,
		};

		//// https://rootllama.wordpress.com/2014/06/20/ray-line-segment-intersection-test-in-2d/
		//// Returns QNaN if intersection is not valiad
		//// Otherwise returns the (persudo) distance t, always satisfy Origin + t*Dir = intersection point  
		//inline XMVECTOR XM_CALLCONV RayIntersects2D(FXMVECTOR Origin, FXMVECTOR Dir, FXMVECTOR A0, GXMVECTOR A1)
		//{
		//	XMVECTOR v1 = Origin - A0;
		//	XMVECTOR v2 = A1 - A0;
		//	XMVECTOR v3 = XMVectorSwizzle<1, 0, 2, 3>(Dir) * g_XMNegateX.v;

		//	XMVECTOR D = XMVectorReciprocal(XMVector2Dot(v2, v3));
		//	XMVECTOR t1 = XMVector2Cross(v2, v1);
		//	XMVECTOR t2 = XMVector2Dot(v1, v3);

		//	//XMVECTOR mask = XMVectorGreater(t2, XMVectorZero());
		//	//// t1 = dot(v1,v3) > 0 ? t1 : -t1;
		//	//t1 = XMVectorXorInt(t1, XMVectorAndInt(mask, XMVectorReplicate(-0.f)));
		//	t1 = XMVectorMultiply(t1, D);
		//	t2 = XMVectorMultiply(t2, D);

		//	// test for t1 > 0 && t2 > 0 && t2 <= 1
		//	XMVECTOR mask = XMVectorGreaterOrEqual(t1, XMVectorZero());
		//	mask = XMVectorAndInt(mask, XMVectorGreaterOrEqual(t2, XMVectorZero()));
		//	mask = XMVectorAndInt(mask, XMVectorLessOrEqual(t2, XMVectorSplatOne()));

		//	//t1 = XMVectorMultiplyAdd(t1,v3,Origin);
		//	t1 = XMVectorSelect(g_XMQNaN.v, t1, mask);
		//	return t1;
		//}

		//inline XMVECTOR XM_CALLCONV LineSegmentIntersects2D(FXMVECTOR A0, FXMVECTOR A1, FXMVECTOR B0, GXMVECTOR B1)
		//{
		//	XMVECTOR dir = A1 - A0;
		//	XMVECTOR t = RayIntersects2D(A0, dir, B0, B1);
		//	XMVECTOR mask = XMVectorLess(t, XMVectorZero());
		//	t = _DXMEXT XMVectorMultiplyAdd(t, dir, A0);
		//	t = XMVectorSelect(g_XMQNaN, t, mask);
		//	return t;
		//}

	}
}

namespace Geometrics
{
	using DirectX::Vector3;
	using DirectX::XMVECTOR;
	using DirectX::XMFLOAT4A;
	using DirectX::FXMVECTOR;

	template <class T>
	void laplacian_smooth(gsl::span<T> curve, float alpha/* = 0.8f*/, unsigned IterationTimes /*= 1*/, bool close_loop /*=false*/)
	{
		unsigned int N = curve.size();
		std::vector<T> cache(N);
		T* BUFF[2] = { &curve[0],&cache[0] };
		unsigned int src = 0, dst = 1;
		float invAlpha = 0.5f * (1.0f - alpha);

		for (unsigned int k = 0; k < IterationTimes; k++)
		{
			if (!close_loop)
			{
				BUFF[dst][0] = BUFF[src][0];
				BUFF[dst][N - 1] = BUFF[src][N - 1];
			}
			else
			{
				BUFF[dst][0] = alpha * BUFF[src][0] + invAlpha * (BUFF[src][N - 1] + BUFF[src][1]);
				BUFF[dst][N - 1] = alpha * BUFF[src][0] + invAlpha * (BUFF[src][N - 2] + BUFF[src][0]);
			}

			for (unsigned int i = 1; i < N - 1; i++)
			{
				BUFF[dst][i] = alpha * BUFF[src][i] + invAlpha * (BUFF[src][i - 1] + BUFF[src][i + 1]);
			}
			dst = !dst;
			src = !src;
		}
		if (dst == 0)
		{
			for (unsigned int i = 0; i < N; i++)
			{
				BUFF[0][i] = BUFF[1][i];
			}
		}
	}

	namespace detail
	{
		template
		<typename _Ty, size_t _Dim>
		struct curve_vertex
		{

		};
	}

	template <typename _Ty, size_t _Dim, typename _TVertex = detail::curve_vertex<_Ty,_Dim>>
	class curve
	{
	};

	// Class to represent a spatial curve with anchor points
	// Provide method for linear sampling from it
	class spatial_curve
	{
	public:
		typedef std::vector<XMFLOAT4A, DirectX::AlignedAllocator<XMFLOAT4A>> AnchorCollection;
		spatial_curve();
		~spatial_curve();
		explicit spatial_curve(const gsl::span<Vector3>& trajectory, bool close_loop = false);

		bool is_close_loop() const { return m_isClose; }
		void set_close(bool close);
		void close_loop() { set_close(true); }

	public:
		int XM_CALLCONV	intersect_ray_2d(FXMVECTOR pos, FXMVECTOR dir, std::vector<DirectX::Vector2>* pContainer = nullptr) const;

		int XM_CALLCONV	intersect_segment_2d(FXMVECTOR A0, FXMVECTOR A1, std::vector<DirectX::Vector2>* pContainer = nullptr) const;

		// check if a given 2D position is contained inside the X-Y projected 2d curve 
		// with the method of casting counting a ray interestion;
		bool XM_CALLCONV contains_2d(FXMVECTOR p, FXMVECTOR test_dir = DirectX::g_XMIdentityR0) const
		{
			return m_isClose && (intersect_ray_2d(p, test_dir) % 2);
		}

		size_t size() const;
		bool empty() const { return m_anchors.empty(); }
		float length() const;
		XMFLOAT4A* data() { return m_anchors.data(); }
		const XMFLOAT4A* data() const { return m_anchors.data(); }
		void clear() { m_anchors.clear(); }
		bool push_back(const Vector3& p, bool force = false);
		bool XM_CALLCONV push_back(FXMVECTOR p, bool force = false);
		bool XM_CALLCONV append(FXMVECTOR p, bool force = false) { return push_back(p,force); }
		XMVECTOR back() const;

		// Retrive the point at parameter position 't' belongs to [0,1]
		// This is O(LogN) level operation
		// perform a binary search in anchors 
		// returns a 4D vector where xyz represent the Position 
		// w is the length from start to point;
		XMVECTOR XM_CALLCONV position(float t) const;
		XMVECTOR XM_CALLCONV position(float t, int hint) const;

		inline XMVECTOR XM_CALLCONV anchor_position(int idx) const {
			using namespace DirectX;
			XMVECTOR v = XMLoadFloat4A(&m_anchors[idx]); 
			v = XMVectorSelect(v, g_XMIdentityR3.v, g_XMIdentityR3.v);
			return v;
		}
		// eval tangent at parameter position t, O(LogN)
		XMVECTOR XM_CALLCONV tangent(float t) const;
		// eval tangent at parameter position t, with a position hint, time-complexity: hint hit O(1), worest case O(LogN)
		XMVECTOR XM_CALLCONV tangent(float t, int hint) const;
		// eval tangent at anchor position idx
		XMVECTOR XM_CALLCONV anchor_tangent(int idx) const;

		inline XMVECTOR XM_CALLCONV operator[](int idx) const { return XMLoadFloat4A(&m_anchors[idx]); }
		inline XMVECTOR XM_CALLCONV operator()(float t) const { return position(t); }


		std::vector<Vector3> sample(size_t sampleCount) const;
		void resample(size_t anchorCount, bool smooth = true);

		void smooth(float alpha/* = 0.8f*/, unsigned iteration /*= 1*/);

		std::vector<Vector3> equidistant_sample(float Interval) const;

		std::vector<Vector3> FixCountSampling(unsigned int SampleSegmentCount, bool Smooth = true) const;
		std::vector<Vector3> FixCountSampling2(unsigned int SampleSegmentCount) const;

		// Update the parameterization information
		void update();

		const XMFLOAT4A& anchor(int idx) const
		{
			return m_anchors[idx];
		}

		XMFLOAT4A& anchor(int idx)
		{
			return m_anchors[idx];
		}

		const AnchorCollection& anchors() const
		{
			return m_anchors;
		}
		AnchorCollection& anchors()
		{
			return m_anchors;
		}

	private:
		// The data we stored is actually aligned on 16-byte boundary , so , use it as a XMFLOAT4A
		AnchorCollection m_anchors;
		bool m_isClose;
	};

}