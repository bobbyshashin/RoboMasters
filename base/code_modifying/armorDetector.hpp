#include "opencv2/highgui/highgui.hpp"
#include "angleSolver.hpp"
#include "settings.hpp"

class ArmorDetector {

public:
	ArmorDetector(const Settings* setting){

		_armorParam = setting->armorParam;



	}

	detectArmor();

private:

	ArmorParam _armorParam;


















};