#pragma once

#include "Pslg2d.h"

namespace M2d
{
	class Parser
	{
	public:
		virtual void parse(const char* filename) = 0;

		Pslg pslg;
	};

	class PolyParser : public Parser
	{
	public:
		void parse(const char* filename);

	private:
		void parseNodes(std::ifstream& file);
		void parseFacets(std::ifstream& file);
		void parseHoles(std::ifstream& file);
		void parseRegions(std::ifstream& file);
	};
}