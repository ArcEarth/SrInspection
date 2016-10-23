#define NOMINMAX
#include "SpaceCurve.h"

using namespace Geometrics;
using namespace DirectX;


spatial_curve::spatial_curve() {
	m_isClose = false;
}

spatial_curve::~spatial_curve() {}

spatial_curve::spatial_curve(const gsl::span<Vector3>& trajectory, bool close_loop/* = false*/)
{
	m_isClose = close_loop;
	for (auto& point : trajectory)
	{
		this->push_back(point);
	}
}


std::vector<Vector3> spatial_curve::FixCountSampling(unsigned int SampleSegmentCount, bool Smooth/* = true*/) const
{
	if (size() <= 1) {
		return std::vector<Vector3>();
	}
	float Interval = length() / SampleSegmentCount;
	// Allocate C+1 size array for storage the result
	std::vector<Vector3> sample(SampleSegmentCount + 1);
	Vector3 ptr;
	float r = 0.0f;
	unsigned int k = 1;

	for (unsigned int i = 0; i < SampleSegmentCount + 1; i++, r += Interval)
	{
		sample[i] = position(r);
	}

	if (Smooth)
		laplacian_smooth<Vector3>(sample, 0.8f, 4, m_isClose);

	return sample;
}

std::vector<Vector3> spatial_curve::FixCountSampling2(unsigned int SampleSegmentCount) const
{
	auto trajectory = FixCountSampling(SampleSegmentCount * 15, true);
	spatial_curve smoothSampler(trajectory);
	return smoothSampler.FixCountSampling(SampleSegmentCount, false);
}

std::vector<Vector3> Geometrics::spatial_curve::sample(size_t sampleCount) const
{
	return FixCountSampling2(sampleCount);
}

void spatial_curve::resample(size_t anchorCount, bool smooth)
{
	auto sample = FixCountSampling2(anchorCount);

	m_anchors.resize(sample.size());
	for (int i = 0; i < sample.size(); i++)
		reinterpret_cast<Vector3&>(m_anchors[i]) = sample[i];

	if (m_isClose)
		m_anchors.push_back(m_anchors[0]);

	update();
}

void Geometrics::spatial_curve::smooth(float alpha, unsigned iteration)
{
	laplacian_smooth<Vector4>( gsl::span<Vector4>((Vector4*)data(), size()), alpha, iteration, m_isClose);
	update();
}

std::vector<Vector3> spatial_curve::equidistant_sample(float Interval) const
{
	assert(Interval != 0.0f);
	float r = length();
	unsigned int Count = (int)(r / Interval) + 1;
	auto trajectory = FixCountSampling(Count * 15, true);
	spatial_curve smoothSampler(trajectory);
	r = smoothSampler.length();
	Count = (int)(r / Interval) + 1;
	return smoothSampler.FixCountSampling(Count, false);
}

void spatial_curve::set_close(bool close) {
	if (!m_isClose && close && !empty())
	{
		auto btr = XMLoadA(m_anchors.back());
		m_anchors.push_back(m_anchors[0]);
		auto vtr = XMLoadA(m_anchors[0]);
		btr = btr - vtr;
		btr += XMVector3Length(btr);
		m_anchors.back().w = XMVectorGetW(btr);
	}

	m_isClose = close; 
}

int XM_CALLCONV Geometrics::spatial_curve::intersect_ray_2d(FXMVECTOR pos, FXMVECTOR dir, std::vector<DirectX::Vector2>* pContainer) const
{
	if (empty()) return 0;
	int count = 0;
	XMVECTOR b1;
	XMVECTOR e1 = XMLoadA(m_anchors[0]);
	for (int i = 0; i < m_anchors.size(); i++)
	{
		b1 = e1;
		e1 = XMLoadA(m_anchors[i]);
		XMVECTOR ip = DirectX::LineSegmentTest::RayIntersects2D(pos, dir, b1, e1);
		if (!DirectX::XMVector4IsNaN(ip))
		{
			++count;
			if (pContainer)
				pContainer->push_back(ip);
		}
	}
	return count;
}

int XM_CALLCONV Geometrics::spatial_curve::intersect_segment_2d(FXMVECTOR A0, FXMVECTOR A1, std::vector<DirectX::Vector2>* pContainer) const
{
	int count = 0;
	XMVECTOR b1;
	XMVECTOR e1 = XMLoadA(m_anchors[0]);
	for (int i = 0; i < m_anchors.size(); i++)
	{
		b1 = e1;
		e1 = XMLoadA(m_anchors[i]);
		XMVECTOR ip = DirectX::LineSegmentTest::LineSegmentIntersects2D(A0, A1, b1, e1);
		if (!DirectX::XMVector4IsNaN(ip))
		{
			++count;
			if (pContainer)
				pContainer->push_back(ip);
		}
	}
	return count;
}

size_t spatial_curve::size() const { 
	return m_anchors.empty() ? 0 : m_anchors.size() - m_isClose; }

float spatial_curve::length() const
{
	if (m_anchors.empty()) return 0.0f;
	return m_anchors.back().w;
}

bool XM_CALLCONV spatial_curve::push_back(FXMVECTOR vtr, bool force)
{
	float len = .0f;
	XMVECTOR vlen, btr, v ;
	v = XMVectorSetW(vtr, .0f);

	if (m_anchors.empty()) {
		m_anchors.emplace_back();
		XMStoreA(m_anchors.back(), v);
	}
	else
	{
		btr = XMLoadA(m_anchors[size() - 1]);
		vlen = _DXMEXT XMVector3Length(v - btr);
		len = XMVectorGetW(vlen);

		// ingnore duplicated anchors
		if (!force && abs(len) < XM_EPSILON * 8)
			return false;

		if (!m_isClose) m_anchors.emplace_back();

		v = _DXMEXT XMVectorSelect<0,0,0,1>(v, vlen + btr);
		XMStoreA(m_anchors.back(), v);
	}

	if (m_isClose)
	{
		btr = v;
		m_anchors.emplace_back();
		XMVECTOR v = XMLoadA(m_anchors[0]);
		vlen = _DXMEXT XMVector3Length(v - btr);
		v = _DXMEXT XMVectorSelect<0, 0, 0, 1>(v, vlen + btr);
		XMStoreA(m_anchors.back(), v);
	}
	return true;
}

XMVECTOR Geometrics::spatial_curve::back() const { return XMLoadFloat4A(&m_anchors[size() - 1]); }

bool spatial_curve::push_back(const Vector3& p,bool force)
{
	return push_back(XMLoad(p),force);
}

// Tangent vector at anchor index
XMVECTOR XM_CALLCONV spatial_curve::position(float t) const
{
	float l = length();
	if (m_isClose)
	{
		t = fmodf(t, l);
		if (t < .0f)
			t += l;
	}
	else
	{
		// clamp to [0,l]
		t = std::max(.0f, std::min(t, l));
	}

	unsigned int a = 0, b = m_anchors.size() - 1;
	while (b - a > 1)
	{
		unsigned int k = (a + b) / 2;
		if (m_anchors[k].w > t) b = k;
		else a = k;
	}

	XMVECTOR v0 = XMLoadFloat4A(&m_anchors[a]);
	XMVECTOR v1 = XMLoadFloat4A(&m_anchors[b]);
	float rt = (t - m_anchors[a].w) / (m_anchors[b].w - m_anchors[a].w);
	XMVECTOR v2 = XMVectorLerp(v0, v1, rt);

	// set w = 1.0
	v2 = XMVectorSelect(v2, g_XMIdentityR3.v, g_XMIdentityR3.v);
	return v2;
}

XMVECTOR XM_CALLCONV spatial_curve::anchor_tangent(int idx) const
{
	using namespace DirectX;
	int pid = idx - (idx > 0);
	int rid = idx + (idx + 1 < m_anchors.size());

	XMVECTOR v0 = XMLoadFloat4A(&m_anchors[pid]);
	XMVECTOR v1 = XMLoadFloat4A(&m_anchors[rid]);
	v1 = XMVectorSubtract(v1, v0);
	v0 = XMVectorSplatW(v1);
	if (XMVector4Less(v0, g_XMEpsilon.v))
	{
		v1 = XMVectorZero();
	}
	else
	{
		v1 = XMVectorDivide(v1, v0);

		// set w = 0 as tangent are pure direction
		v1 = XMVectorSelect(v1, XMVectorZero(), g_XMIdentityR3.v);
	}

	return v1;
}

XMVECTOR XM_CALLCONV spatial_curve::tangent(float t) const
{
	float l = length();
	if (m_isClose)
	{
		t = fmodf(t, l);
		if (t < .0f)
			t += l;
	}
	else
	{
		// clamp to [0,l]
		t = std::max(.0f, std::min(t, l));
	}

	// binary search for parameter
	int a = 0, b = m_anchors.size() - 1;

	while (b - a > 1)
	{
		unsigned int k = (a + b) / 2;
		if (m_anchors[k].w > t) b = k;
		else a = k;
	}

	XMVECTOR v0 = anchor_tangent(a);
	XMVECTOR v1 = anchor_tangent(b);
	float rt = (t - m_anchors[a].w) / (m_anchors[b].w - m_anchors[a].w);
	XMVECTOR v2 = XMVectorLerp(v0, v1, rt);

	return v2;
}

void spatial_curve::update()
{
	m_anchors[0].w = 0;
	XMVECTOR p0 = XMLoadFloat4A(&m_anchors[0]);
	XMVECTOR p1;
	for (int i = 1; i < m_anchors.size(); i++)
	{
		p1 = XMLoadFloat4A(&m_anchors[i]);
		float dt = XMVectorGetX(XMVector3Length(p1 - p0));
		m_anchors[i].w = m_anchors[i - 1].w + dt;
		p0 = p1;
	}
}