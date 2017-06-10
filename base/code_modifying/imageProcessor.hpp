#include "opencv2/opencv.hpp"
#include "armorDetector.hpp"
//#include "runeDetector.hpp"
#include "settings.hpp"

class ImageProcessor {

public:
	ImageProcessor(Settings settings, int fd){

		_settings = settings;
		_fd = fd;

	}

	void imageProducer();
	void imageConsumer();

private:
	Settings* _settings;
	int _fd;
};