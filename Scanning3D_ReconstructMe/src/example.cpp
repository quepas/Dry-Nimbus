#include <reconstructmesdk/reme.h>

#include <sstream>
#include <iostream>

using std::cin;
using std::cout;
using std::endl;
using std::stringstream;

int main () {
  reme_context_t context;
  reme_context_create(&context);

  reme_options_t options, options_sub;
  reme_options_create(context, &options);
  reme_options_create(context, &options_sub);

  reme_error_t error;
  if (reme_context_bind_opencl_info(context, options) != REME_ERROR_SUCCESS) {
    cout << "Error while getting OpenCL compatible devices." << endl;
    return 0;
  }

  int num_devices = 0;
  reme_options_get_repeated_count(context, options, "devices", &num_devices);
  cout << "Found " << num_devices << " OpenCL compatible devices." << endl;
  char name[256], vendor[256], type[256];

  for (int idx = 0; idx < num_devices; ++idx) {
    cout << "---------------------------------" << endl;
    reme_options_bind_repeated_message(context, options, "devices", idx, options_sub);
    reme_options_get(context, options_sub, "name", name, 256);
    reme_options_get(context, options_sub, "vendor", vendor, 256);
    reme_options_get(context, options_sub, "type", type, 256);
    cout << "\tDeviceID: " << idx << endl;
    cout << "\tName: " << name << endl;
    cout << "\tVendor: " << vendor << endl;
    cout << "\tType: " << type << endl;
  }

  reme_options_t main_options;
  reme_options_create(context, &main_options);

  error = reme_context_bind_reconstruction_options(context, main_options);

  int device_id = -1;
  cout << "Enter device ID to use: ";
  cin >> device_id;
  stringstream sstream;
  sstream << device_id;

  reme_options_set(context, main_options, "device_id", sstream.str().c_str());
  reme_context_compile(context);
  reme_context_print_errors(context);

  reme_sensor_t sensor;
  reme_sensor_create(context, "openni", true, &sensor);
  reme_sensor_open(context, sensor);

  reme_image_t image_aux, image_depth, image_volume;
  reme_image_create(context, &image_aux);
  reme_image_create(context, &image_depth);
  reme_image_create(context, &image_volume);

  reme_viewer_t viewer;
  reme_viewer_create_image(context, "Image", &viewer);
  reme_viewer_add_image(context, viewer, image_aux);
  reme_viewer_add_image(context, viewer, image_depth);
  reme_viewer_add_image(context, viewer, image_volume);

  while (REME_SUCCESS(reme_sensor_grab(context, sensor))) {
    if (REME_SUCCESS(reme_sensor_track_position(context, sensor))) {
      reme_sensor_update_volume(context, sensor);
    }
    reme_sensor_prepare_images(context, sensor);
    reme_sensor_get_image(context, sensor, REME_IMAGE_AUX, image_aux);
    reme_sensor_get_image(context, sensor, REME_IMAGE_DEPTH, image_depth);
    reme_sensor_get_image(context, sensor, REME_IMAGE_VOLUME, image_volume);
    reme_viewer_update(context, viewer);
  }
  reme_context_print_errors(context);
  reme_context_destroy(&context);
}
