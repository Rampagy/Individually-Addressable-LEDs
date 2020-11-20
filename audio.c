#include "audio.h"

void vInitAudio( void )
{
    //arm_status status;
    arm_rfft_fast_instance_f32 S;
    //float32_t maxValue;
    //status = ARM_MATH_SUCCESS;

    (void) arm_rfft_fast_init_f32( &S, 256 );
    //(void) arm_cfft_radix4_init_f32();
}