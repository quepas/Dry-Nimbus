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
}
