#pragma once

#include <Enums.hpp>
#include <cmath>

namespace sb
{
	enum class Easing
	{
		None,
		Out,
		In,
		InQuad,
		OutQuad,
		InOutQuad,
		InCubic,
		OutCubic,
		InOutCubic,
		InQuart,
		OutQuart,
		InOutQuart,
		InQuint,
		OutQuint,
		InOutQuint,
		InSine,
		OutSine,
		InOutSine,
		InExpo,
		OutExpo,
		InOutExpo,
		InCirc,
		OutCirc,
		InOutCirc,
		InElastic,
		OutElastic,
		OutElasticHalf,
		OutElasticQuarter,
		InOutElastic,
		InBack,
		OutBack,
		InOutBack,
		InBounce,
		OutBounce,
		InOutBounce,
		Step
	};

	constexpr double PI = 3.14159265358979323846;
	double Reverse(double(*f)(double), double t) { return 1 - f(1 - t); }
	double ToInOut(double(*f)(double), double t) { return 0.5 * (t < 0.5 ? f(2 * t) : (2 - f(2 - 2 * t))); }
	double Step(double t) { return t >= 1 ? 1 : 0; }
	double Linear(double t) { return t; }
	double InQuad(double t) { return t * t; }
	double OutQuad(double t) { return Reverse(InQuad, t); }
	double InOutQuad(double t) { return ToInOut(InQuad, t); }
	double InCubic(double t) { return t * t * t; }
	double OutCubic(double t) { return Reverse(InCubic, t); }
	double InOutCubic(double t) { return ToInOut(InCubic, t); }
	double InQuart(double t) { return t * t * t * t; }
	double OutQuart(double t) { return Reverse(InQuart, t); }
	double InOutQuart(double t) { return ToInOut(InQuart, t); }
	double InQuint(double t) { return t * t * t * t * t; }
	double OutQuint(double t) { return Reverse(InQuint, t); }
	double InOutQuint(double t) { return ToInOut(InQuint, t); }
	double InSine(double t) { return 1 - std::cos(t * PI / 2); }
	double OutSine(double t) { return Reverse(InSine, t); }
	double InOutSine(double t) { return ToInOut(InSine, t); }
	double InExpo(double t) { return std::pow(2, 10 * (t - 1)); }
	double OutExpo(double t) { return Reverse(InExpo, t); }
	double InOutExpo(double t) { return ToInOut(InExpo, t); }
	double InCirc(double t) { return 1 - std::sqrt(1 - t * t); }
	double OutCirc(double t) { return Reverse(InCirc, t); }
	double InOutCirc(double t) { return ToInOut(InCirc, t); }
	double InBack(double t) { return t * t * ((1.70158 + 1) * t - 1.70158); }
	double OutBack(double t) { return Reverse(InBack, t); }
	double InOutBack(double t) { return ToInOut([](double y) { return y * y * ((1.70158 * 1.525 + 1) * y - 1.70158 * 1.525); }, t); }
	double OutBounce(double t) { return t < 1 / 2.75 ? 7.5625 * t * t : t < 2 / 2.75 ? 7.5625 * (t -= (1.5 / 2.75)) * t + .75 : t < 2.5 / 2.75 ? 7.5625 * (t -= (2.25 / 2.75)) * t + .9375 : 7.5625 * (t -= (2.625 / 2.75)) * t + .984375; }
	double InBounce(double t) { return Reverse(OutBounce, t); }
	double InOutBounce(double t) { return ToInOut(InBounce, t); }
	double OutElastic(double t) { return std::pow(2, -10 * t) * std::sin((t - 0.075) * (2 * t) / .3) + 1; }
	double InElastic(double t) { return Reverse(OutElastic, t); }
	double OutElasticHalf(double t) { return std::pow(2, -10 * t) * std::sin((0.5 * t - 0.075) * (2 * PI) / .3) + 1; }
	double OutElasticQuarter(double t) { return std::pow(2, -10 * t) * std::sin((0.25 * t - 0.075) * (2 * PI) / .3) + 1; }
	double InOutElastic(double t) { return ToInOut(InElastic, t); }

	double applyEasing(Easing easing, double t)
	{
		switch (easing)
		{
		case sb::Easing::Step: return Step(t); break;
		case sb::Easing::None: return Linear(t); break;
		case sb::Easing::Out: return OutQuad(t); break;
		case sb::Easing::In: return InQuad(t); break;
		case sb::Easing::InQuad: return InQuad(t); break;
		case sb::Easing::OutQuad: return OutQuad(t); break;
		case sb::Easing::InOutQuad: return InOutQuad(t); break;
		case sb::Easing::InCubic: return InCubic(t); break;
		case sb::Easing::OutCubic: return OutCubic(t); break;
		case sb::Easing::InOutCubic: return InOutCubic(t); break;
		case sb::Easing::InQuart: return InQuart(t); break;
		case sb::Easing::OutQuart: return OutQuart(t); break;
		case sb::Easing::InOutQuart: return InOutQuart(t); break;
		case sb::Easing::InQuint: return InQuint(t); break;
		case sb::Easing::OutQuint: return OutQuint(t); break;
		case sb::Easing::InOutQuint: return InOutQuint(t); break;
		case sb::Easing::InSine: return InSine(t); break;
		case sb::Easing::OutSine: return OutSine(t); break;
		case sb::Easing::InOutSine: return InOutSine(t); break;
		case sb::Easing::InExpo: return InExpo(t); break;
		case sb::Easing::OutExpo: return OutExpo(t); break;
		case sb::Easing::InOutExpo: return InOutExpo(t); break;
		case sb::Easing::InCirc: return InCirc(t); break;
		case sb::Easing::OutCirc: return OutCirc(t); break;
		case sb::Easing::InOutCirc: return InOutCirc(t); break;
		case sb::Easing::InElastic: return InElastic(t); break;
		case sb::Easing::OutElastic: return OutElastic(t); break;
		case sb::Easing::OutElasticHalf: return OutElasticHalf(t); break;
		case sb::Easing::OutElasticQuarter: return OutElasticQuarter(t); break;
		case sb::Easing::InOutElastic: return InOutElastic(t); break;
		case sb::Easing::InBack: return InBack(t); break;
		case sb::Easing::OutBack: return OutBack(t); break;
		case sb::Easing::InOutBack: return InOutBack(t); break;
		case sb::Easing::InBounce: return InBounce(t); break;
		case sb::Easing::OutBounce: return OutBounce(t); break;
		case sb::Easing::InOutBounce: return InOutBounce(t); break;
		default: return Linear(t); break;
		}
	}

	template <typename T>
	T InterpolateLinear(T start, T end, double t)
	{
		return start + (end - start) * t;
	}

	std::pair<double, double> InterpolateLinear(std::pair<double, double> start, std::pair<double, double> end, double t)
	{
		return std::pair<double, double>(InterpolateLinear(start.first, end.first, t), InterpolateLinear(start.second, end.second, t));
	}

	template <typename T>
	T InterpolateBilinear(T topLeft, T topRight, T bottomLeft, T bottomRight, double tx, double ty)
	{
		return InterpolateLinear(
			InterpolateLinear(topLeft, topRight, tx),
			InterpolateLinear(bottomLeft, bottomRight, tx),
			ty
		);
	}

	/*
	template <typename T>
	T InterpolateBilinear(T topLeft, T topRight, T bottomLeft, T bottomRight, double tx, double ty)
	{
		return (topLeft + (topRight - topLeft) * tx) * (1 - ty) + (bottomLeft + (bottomRight - bottomLeft) * tx) * ty;
	}
	*/
}