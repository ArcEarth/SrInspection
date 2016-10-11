#include <Geometrics\bvh.h>
#include <hlslm\hlsl.hpp>
#include <iostream>
#include <thread>


struct XM_ALIGNATTR aabb_t {
	hlsl::xmvector3f min;
	hlsl::xmvector3f max;
	template <size_t dim>
	static bool less(const aabb_t& lhs, const aabb_t & rhs)
	{
		using namespace hlsl;
		return hlsl::less(lhs.min, rhs.min).get<dim>();
	}

	aabb_t operator+(const aabb_t& rhs) const
	{
		const aabb_t& lhs = *this;
		using namespace hlsl;
		aabb_t ret{ hlsl::min(lhs.min,rhs.min),hlsl::max(lhs.max,rhs.max) };
		return ret;
	}
};

template <typename _Scalar, size_t _Dim>
std::ostream& operator<<(std::ostream& lhs, hlsl::xmvector<_Scalar, _Dim> v)
{
	for (int i = 0; i < _Dim; i++)
	{
		lhs << v.get(i) << ',';
	}
	lhs << "\b";
	return lhs;
}

using namespace Geometrics;
using namespace std;

using tree_t = kd_tree<int, float, 3, aabb_t, vector<int>&>;

struct XM_ALIGNATTR aabb_ray_intersector {
	DirectX::XMVECTOR origin;
	DirectX::XMVECTOR direction;
	int n;
	int &counter;
	float XM_CALLCONV operator ()(const aabb_t& aabb) const
	{
		counter++;
		const float epsilon = 0.001f;
		using namespace hlsl;
		DirectX::BoundingBox box;
		xmvector3f extent = (aabb.max - aabb.min) * 0.5f;
		extent = hlsl::max(extent, xmfloat(epsilon));
		box.Center << (aabb.max + aabb.min) * 0.5f;
		box.Extents << extent;
		float f;
		bool succ = box.Intersects(origin, direction, f);
		// if (succ)
		// std::cout << (succ ? 'O' : 'X') << " [" << aabb.min << " - " << aabb.max << "]" << std::endl;
		if (succ) return f;
		else return -1.0f;
	}

	float XM_CALLCONV operator ()(int obj) const
	{
		int i = (int)obj;
		int j = i % n;
		i = i / n;
		float fi = i, fj = j;
		tree_t::volumn_t aabb{ { fi,fj,.0f },{ fi + 1,fj + 1,0.f } };
		return this->operator()(aabb);
	}
};

int main(/*int argc, const char**argv*/)
{

	int n = 10;
	vector<int> points(n*n);
	for (int i = 0; i < points.size(); i++)
	{
		points[i] = i;
	}



	tree_t tree(points, [n](const int &obj) ->tree_t::volumn_t {
		int i = (int)obj;
		int j = i % n;
		i = i / n;
		float fi = i, fj = j;
		return tree_t::volumn_t{ {fi,fj,.0f},{fi+1,fj+1,0.f} };
	});

	tree.rebuild();

	//for (int i = 0; i < 1000; i++)
	//{
	//	tree.rebuild();
	//}

	int counter = 0;
	DirectX::XMVECTOR v = hlsl::normalize(hlsl::xmvector3f(1.0f, 1.0f, 0.f));
	auto ray_intersector = aabb_ray_intersector{ { -1.51f,-1.50f,0.0f,0.f },v, n, counter };
	auto itr = find_first_of(tree, ray_intersector, ray_intersector);
	std::cout << "Itersection with x = 3.5, y = 3.5 :" << std::endl;
	if (itr)
	{
		int s = (int)(*itr);
		std::cout << '(' << s / n << ',' << s % n <<"), with " << counter << " times intersection test" << std::endl;
	} else
	{ 
		std::cout << " Not found!" << std::endl;
	}

	system("PAUSE");
	//auto func = [] {
	//	system("PAUSE");
	//	std::cout << this_thread::get_id() << std::endl; };
	//thread t0(func);
	//thread t1(func);
	//thread t2(func);
	//thread t3(func);
	//t0.join(); t1.join(); t2.join(); t3.join();
}