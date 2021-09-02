
#include <iostream>
#include <fcntl.h>
#include <unistd.h>

#include "tinybson.h"

using namespace std;

static int
param_import_callback(bson_decoder_t decoder, void *priv, bson_node_t node)
{
	float f = 0.0f;
	int32_t i = 0;
	void *tmp = nullptr;
	void *v = nullptr;
	int result = -1;

	/*
	 * EOO means the end of the parameter object. (Currently not supporting
	 * nested BSON objects).
	 */
	if (node->type == BSON_EOO) {
		return 0;
	}

	switch (node->type) {
	case BSON_BOOL:
		cout << node->name << ",boolean," << node->b << endl;
		break;

	case BSON_INT32:
		cout << node->name << ",int32," << node->i << endl;
		break;

	case BSON_INT64:
		cout << node->name << ",int64," << node->i << endl;
		break;

	case BSON_DOUBLE:
		cout << node->name << ",double," << node->d << endl;
		break;

	case BSON_STRING:
        cerr << "Error: Unsupported type STRING" << endl;
		break;

	case BSON_BINDATA:
        cerr << "Error: Unsupported type BINDATA" << endl;
		break;

	/* XXX currently not supporting other types */
	default:
        cerr << "Error: unknown data type" << endl;
	}
}

int main() {
    cout << "param-test starting" << endl;

    const char filename[] = "parameters";

    int fd = open(filename, O_RDONLY);
    if (fd < 0) {
        cerr << "Error: Couldn't open file " << filename << endl;
        return -1;
    }

	bson_decoder_s decoder;

	if (bson_decoder_init_file(&decoder, fd, param_import_callback, nullptr)) {
        cerr << "Error: Couldn't initialize parameter file decoder" << endl;
    } else {
	   int result = -1;

    	do {
    		result = bson_decoder_next(&decoder);
    	} while (result > 0);
    }

    close(fd);

    return 0;
}
