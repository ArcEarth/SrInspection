#include <Geometrics\KdBVH.h>
#include <hlslm\hlsl.hpp>
#include <iostream>

int main(/*int argc, const char**argv*/)
{
	using namespace Geometrics;
	using namespace std;

	int n = 1000;
	vector<int> points(n*n);
	for (int i = 0; i < points.size(); i++)
	{
		points[i] = i;
	}

	using tree_t = kd_tree<int, float, 2>;



	tree_t tree(points.begin(), points.end(), [n](const int &obj) ->tree_t::aabb_t {
		int i = (int)obj;
		int j = i % n;
		i = i / n;
		float fi = i, fj = j;
		return tree_t::aabb_t{ {fi,fj},{fi+1,fj+1} };
	});

	for (int i = 0; i < 1000; i++)
	{
		tree.rebuild();
	}

	int counter = 0;

	auto itr = find_all_if(tree, [&counter](const tree_t::aabb_t& vol) -> bool {
		counter++;
		bool succ = vol.min[0] < 3.5f && 3.5f < vol.max[0] &&
			vol.min[1] < 3.5f && 3.5f < vol.max[1];
		//if (succ)
		//	std::cout << "testing [" << vol.min[0] << '-' << vol.max[0] << ',' << vol.min[1] << '-' << vol.max[1] << ']' << endl;

		return succ;
	});
	std::cout << "Itersection with x = 3.5, y = 3.5 :" << std::endl;
	while (itr)
	{
		int s = (int)(*itr);
		std::cout << '(' << s / 10 << ',' << s % 10 <<"), with " << counter << " times intersection test" << std::endl;
		itr++;
	}
	system("PAUSE");
}