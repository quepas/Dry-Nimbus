#include <reconstructmesdk/reme.h>

#include <iostream>
#include <sstream>

bool SelectOpenCLCompatibleDevice(reme_context_t context) {
  reme_options_t options, new_options;
  reme_options_create(context, &options);
  reme_options_create(context, &new_options);

  reme_error_t error;
  if (reme_context_bind_opencl_info(context, options) != REME_ERROR_SUCCESS) {
    std::cout << "Error while getting OpenCL compatible devices." << std::endl;
    return false;
  }

  int num_devices = 0;
  reme_options_get_repeated_count(context, options, "devices", &num_devices);
  std::cout << "Found " << num_devices << " OpenCL compatible devices." << std::endl;
  char name[256], vendor[256], type[256];

  for (int idx = 0; idx < num_devices; ++idx) {
    std::cout << "---------------------------------" << std::endl;
    reme_options_bind_repeated_message(context, options, "devices", idx, new_options);
    reme_options_get(context, new_options, "name", name, 256);
    reme_options_get(context, new_options, "vendor", vendor, 256);
    reme_options_get(context, new_options, "type", type, 256);
    std::cout << "\tDeviceID: " << idx << std::endl;
    std::cout << "\tName: " << name << std::endl;
    std::cout << "\tVendor: " << vendor << std::endl;
    std::cout << "\tType: " << type << std::endl;
  }
  //reme_options_t main_options;
  reme_options_create(context, &options);

  error = reme_context_bind_reconstruction_options(context, options);
  int device_id = -1;
  std::cout << "Enter device ID to use: ";
  std::cin >> device_id;
  std::stringstream sstream;
  sstream << device_id;

  reme_options_set(context, options, "device_id", sstream.str().c_str());
  reme_context_compile(context);
  reme_context_print_errors(context);
  return true;
}

int main() {
  reme_context_t context;
  reme_context_create(&context);

  if (!SelectOpenCLCompatibleDevice(context)) {
    return EXIT_FAILURE;
  }

  // create volume
  reme_volume_t volume;
  reme_volume_create(context, &volume);

  
  // create sensor
  reme_sensor_t sensor;
  reme_sensor_create(context, "openni;mskinect", true, &sensor);
  reme_sensor_open(context, sensor);
  reme_sensor_set_prescan_position(context, sensor, REME_SENSOR_POSITION_INFRONT);

  // grab frames and fill volume
  int count = 0;
  while (count++ < 200 && REME_SUCCESS(reme_sensor_grab(context, sensor))) {
    if (REME_SUCCESS(reme_sensor_track_position(context, sensor))) {
      reme_sensor_update_volume(context, sensor);
    }
  }

  // destroy sensor
  reme_sensor_close(context, sensor);
  reme_sensor_destroy(context, &sensor);

  reme_surface_t surface;
  reme_surface_create(context, &surface);

  reme_options_t options;
  reme_options_create(context, &options);
  reme_surface_bind_generation_options(context, surface, options);

  reme_options_set_bool(context, options, "merge_duplicate_vertices", false);
  reme_surface_generate(context, surface, volume);
  reme_surface_bind_poisson_options(context, surface, options);

  reme_options_set_int(context, options, "depth", 8);
  reme_surface_poisson(context, surface);

  // reduce mesh triangles
  reme_surface_bind_decimation_options(context, surface, options);
  reme_options_set_int(context, options, "maximum_faces", 20000);
  reme_surface_decimate(context, surface);

  // display surface
  reme_viewer_t viewer;
  reme_viewer_create_surface(context, surface, "Surface viewer", &viewer);
  reme_viewer_wait(context, viewer);

  // print errors & destroy context
  reme_context_print_errors(context);
  reme_context_destroy(&context);

  getchar();
}