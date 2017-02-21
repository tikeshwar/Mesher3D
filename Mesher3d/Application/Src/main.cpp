#include <windows.h>
#include <iostream>
#include <ctime> 

//#undef _ITERATOR_DEBUG_LEVEL
//#define	_ITERATOR_DEBUG_LEVEL 0

#include <vector> 

#include "Defines.h"
#include "BBox.h"
#include "Mat4x4.h"

//#define dim 1

//#ifdef dim

#include "Parser2d.h"
#include "Mesher2d.h"

//#endif

//#ifndef dim

#include "Parser.h"
#include "CDTPslg.h"
#include "Mesher3d.h"

//#endif


using namespace GCore;

int main()
{
//#ifndef dim
	{
		//M3d::PolyParser parser;
		//parser.parse("C:/Users/Tikeshwar/Desktop/polyfiles/example.poly");

		M3d::OffParser parser;
		parser.parse("C:/Users/Tikeshwar/Desktop/polyfiles/cylinder.off");

		//M3d::SMeshParser parser;
		//parser.parse("C:/Users/Tikeshwar/Desktop/polyfiles/cross.smesh");

		//M3d::CDTPslg cdtpslg(parser.pslg);
		//cdtpslg.createCDT();

		//cdtpslg.writeToOff("cdtplc.off");
		//system("start meshlab.exe cdtplc.off");

		//M3d::Mesher3d mesher(cdtpslg.cdtPslg);
		M3d::Mesher3d mesher(parser.pslg);
		mesher.createDelaunay3D();
		mesher.writeToOff("cubeTet.off");
	}
//#endif

//#ifdef dim
	/*{
		M2d::PolyParser parser;
		parser.parse("C:/Users/Tikeshwar/Desktop/polyfiles/pie_2d.poly");

		M2d::Mesher2d mesher(parser.pslg);
		mesher.createDelaunay2D();
		mesher.writeToOff("cubeTet.off");
	}*/
//#endif

	system("start meshlab.exe cubeTet.off");
	return 1;
}