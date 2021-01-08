#pragma once

#include <unordered_map>
#include <string>

namespace sb
{
	enum class Layer
	{
		Background,
		Fail,
		Pass,
		Foreground,
		Overlay
	};
	static const std::unordered_map<std::string, Layer> LayerStrings =
	{
		{"Background", Layer::Background},
		{"Fail", Layer::Fail},
		{"Pass", Layer::Pass},
		{"Foreground", Layer::Foreground},
		{"Overlay", Layer::Overlay}
	};

	enum class Origin
	{
		TopLeft,
		TopCentre,
		TopRight,
		CentreLeft,
		Centre,
		CentreRight,
		BottomLeft,
		BottomCentre,
		BottomRight
	};
	static const std::unordered_map<std::string, Origin> OriginStrings =
	{
		{"TopLeft", Origin::TopLeft},
		{"TopCentre", Origin::TopCentre},
		{"TopRight", Origin::TopRight},
		{"TopRight", Origin::TopRight},
		{"CentreLeft", Origin::CentreLeft},
		{"Centre", Origin::Centre},
		{"CentreRight", Origin::CentreRight},
		{"BottomLeft", Origin::BottomLeft},
		{"BottomCentre", Origin::BottomCentre},
		{"BottomRight", Origin::BottomRight}
	};

	enum class LoopType
	{
		LoopForever,
		LoopOnce,
		Custom
	};
	static const std::unordered_map<std::string, LoopType> LoopTypeStrings =
	{
		{"LoopForever", LoopType::LoopForever},
		{"LoopOnce", LoopType::LoopOnce}
	};

	enum class ParameterType
	{
		FlipH,
		FlipV,
		Additive
	};
	static const std::unordered_map<std::string, ParameterType> ParameterTypeStrings =
	{
		{"H", ParameterType::FlipH},
		{"V", ParameterType::FlipV},
		{"A", ParameterType::Additive}
	};

	enum class EventType
	{
		None,
		F,
		S,
		V,
		R,
		M,
		MX,
		MY,
		C,
		P
	};
	static const std::unordered_map<std::string, EventType> EventTypeStrings
	{
		{"F", EventType::F},
		{"S", EventType::S},
		{"V", EventType::V},
		{"R", EventType::R},
		{"M", EventType::M},
		{"MX", EventType::MX},
		{"MY", EventType::MY},
		{"C", EventType::C},
		{"P", EventType::P}
	};

	enum class Keyword
	{
		None,
		Sprite,
		Animation,
		Sample,
		T,
		L
	};
	static const std::unordered_map<std::string, Keyword> KeywordStrings =
	{
		{"Sprite", Keyword::Sprite},
		{"Animation", Keyword::Animation},
		{"Sample", Keyword::Sample},
		{"T", Keyword::T},
		{"L", Keyword::L},
	};
}