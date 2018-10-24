#include <libcamera.h>

int main(void)
{
	libcamera l = libcamera();
	l.init_lib();

	return 0;
}
