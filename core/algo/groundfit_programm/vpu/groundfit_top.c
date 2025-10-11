/** [common_params] */

/** [common_params] */

/* Let's declare buffers that will hold the input and output tiles of the image data.
 * RDF_SINGLE helper macro can be used to get the VMEM size needed to store one tile.
 * Type, width and height of the 2-D image tile are passed as arguments.
 * There are also some optional macro arguments, which will be introduced in the upcoming tutorials.
 * RDF requires certain size alignment and the helper macros will ensure that requirements are satisfied.
 *
 * The number of tiles in the image is set by the host code.
 * We will use the tileCount variable to set the number of outer loop iterations.
 */
#include <cupva_device.h> /* Main device-side header file */
#include "../groundfit_common_param.h"



CUPVA_VPU_MAIN()
{
    return 0;
}
/** [release_and_close] */

