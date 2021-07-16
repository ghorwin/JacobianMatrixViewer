#ifndef IBKMK_TriangulationH
#define IBKMK_TriangulationH

#include <IBK_point.h>
#include <IBK_assert.h>

namespace IBKMK {

/*! Performs triangulation.
	This class wraps the internal triangulation library so that users do not need to know
	the details of the underlying library's API.
*/
class Triangulation {
public:
	/*! Simple storage member to hold vertex indexes of a single triangle. */
	struct triangle_t {
		triangle_t() {}
		triangle_t(unsigned int n1, unsigned int n2, unsigned int n3) :
			i1(n1), i2(n2), i3(n3)
		{}

		unsigned int i1=0, i2=0, i3=0;
	};

	/*! Set points to triangulate.
		No duplicate points (within tolerance allowed!)
		Also, edges must mark outer and inner boundaries of surface.
	*/
	bool setPoints(const std::vector<IBK::point2D<double> > & points,
				   const std::vector<std::pair<unsigned int, unsigned int> > & edges);

	/*! Tolerance criterion - points within this distances are
		takes and "same".
	*/
	double	m_tolerance;

	/*! Contains the generated triangles after triangulation has completed. */
	std::vector<triangle_t>		m_triangles;
};

} // namespace IBKMK

#endif // IBKMK_TriangulationH
