#include "IBK_FluidPhysics.h"

#include "IBK_assert.h"

#include <cmath>

/*! Reynolds number where flow switches from laminar to transition state. */
#define RE_LAMINAR		2300.0

/*! Reynolds number where flow switches from transition state to turbulent */
#define RE_TURBULENT	4000.0

namespace IBK {


double PipeFrictionFactor(const double & reynolds, const double & d, const double & roughness) {
	if (reynolds < RE_LAMINAR)
		return 64.0/reynolds;
	else if (reynolds < RE_TURBULENT){
		double fLam = 64.0/RE_LAMINAR;
		double fTurb = FrictionFactorColebrook(RE_TURBULENT, roughness / d);
		// now interpolate linearly between fLam and fTurb
		return fLam + (reynolds - RE_LAMINAR) * (fTurb - fLam) / (RE_TURBULENT - RE_LAMINAR);
	}
	else{
		return	FrictionFactorColebrook(reynolds, roughness / d);
	}
}


double FrictionFactorSwamee(const double &reynolds, const double &d, const double &roughness) {
	if (reynolds < RE_LAMINAR)
		return 64.0/reynolds;
	else if (reynolds < RE_TURBULENT){
		double fLam = 64.0/RE_LAMINAR; // f(RE_LAMINAR)
		double fTurb = std::log10((roughness / d) / 3.7 + 5.74 / std::pow(RE_TURBULENT, 0.9) );
		fTurb = 0.25/(fTurb*fTurb); // f(RE_TURBULENT)
		// now interpolate linearly between fLam and fTurb
		return fLam + (reynolds - RE_LAMINAR) * (fTurb - fLam) / (RE_TURBULENT - RE_LAMINAR);
	}
	else{
		double f = std::log10( (roughness / d) / 3.7 + 5.74 / std::pow(reynolds, 0.9) ) ;
		return	0.25 / (f*f);
	}
}


double fast_log10(double x) {
	return std::log(x) * 0.4342944819032518; // 1/ln(10)
}

double FrictionFactorColebrook(double reynolds, double relativeRoughness, int maxIterations, double tolerance) {
	// Serghides' explicit approximation as initial guess
	double A = -2 * fast_log10(relativeRoughness / 3.7 + 12 / reynolds);
	double B = -2 * fast_log10(relativeRoughness / 3.7 + 2.51 * A / reynolds);
	double C = -2 * fast_log10(relativeRoughness / 3.7 + 2.51 * B / reynolds);

	double f = std::pow((A - (B - A) * (B - A) / (C - 2 * B + A)), -2);

	// Precompute constants
	const double a = relativeRoughness / 3.7;
	const double b = 2.51 / reynolds;

	for (int i = 0; i < maxIterations; ++i) {
		double f_sqrt = std::sqrt(f);
		double rhs = a + b / f_sqrt;
		double f_new = 0.25 / std::pow(fast_log10(rhs), 2);

		if (std::abs(f - f_new) < tolerance) {
			return f_new;
		}

		f = f_new;
	}

	return f;
}


double NusseltNumber(const double &reynolds, const double &prandtl, const double &l, const double &d) {
	if (reynolds < RE_LAMINAR){
		return NusseltNumberLaminar(reynolds, prandtl, l, d);
	}
	else if (reynolds < RE_TURBULENT){
		double nuLam = NusseltNumberLaminar(RE_LAMINAR, prandtl, l, d);
		double nuTurb = NusseltNumberTurbulent(RE_TURBULENT, prandtl, l, d);
		return nuLam + (reynolds - RE_LAMINAR) * (nuTurb - nuLam) / (RE_TURBULENT - RE_LAMINAR);
	}
	else {
		return NusseltNumberTurbulent(reynolds, prandtl, l, d);
	}
}


double NusseltNumberTurbulent(const double &reynolds, const double &prandtl, const double &l, const double &d) {
	if (reynolds <=0)
		return 3.66; // for velocity=0
	// Equation 25, VDI-Wärmeatlas (11. Auflage), Kapitel B2, Page 28
	double zeta = std::pow(1.8 * std::log10(reynolds) - 1.5, -2.0);
	return zeta / 8. * reynolds*prandtl /
		(1. + 12.7 * std::sqrt(zeta / 8.) * (std::pow(prandtl, 0.6667) - 1.)) *
							 (1. + std::pow(d / l, 0.6667));
}


double NusseltNumberLaminar(const double &reynolds, const double &prandtl, const double &l, const double &d) {
	if (reynolds <=0)
		return 3.66; // for velocity=0
	else
		return std::pow( 49.37 + std::pow(1.615 * std::pow(reynolds * prandtl * d/l, 1.0/3.0) - 0.7, 3.0) , 1.0/3.0);
}


double PrandtlNumber(const double &kinVis, const double &cp, const double &lambda, const double &rho) {
	return kinVis * cp * rho / lambda;
}


double ReynoldsNumber(const double &v, const double &kinVis, const double &d) {
	return  v * d / kinVis;
}


} // namespace IBK
