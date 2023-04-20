/*
 * max30100_filter.c
 *
 *  Created on: 11 апр. 2023 г.
 *      Author: Dmitry
 */

#include "max30100.h"


float dc_remove(float x, float prev_w, float alpha){
	float return_value;
	return_value = x + alpha * prev_w;
	return_value = return_value - prev_w;
	return return_value;
}

float mean_diff(float present_value, meanDiffFilter_t* filterValues){
	float average_value;

	filterValues->sum -= filterValues->values[filterValues->index];
	filterValues->values[filterValues->index] = present_value;
	filterValues->sum += filterValues->values[filterValues->index];

	filterValues->index++;
	filterValues->index = filterValues->index % MEAN_FILTER_SIZE;

	if(filterValues->count < MEAN_FILTER_SIZE)
		filterValues->count++;

	average_value = filterValues->sum / filterValues->count;
	return average_value - present_value;
}

void butterworth_filter(float x, butterworthFilter_t * filterResult){
	filterResult->value[0] = filterResult->value[1];

	//Fs = 100Hz and Fc = 10Hz
	filterResult->value[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * filterResult->value[0]);

	//Fs = 100Hz and Fc = 4Hz
	//filterResult->v[1] = (1.367287359973195227e-1 * x) + (0.72654252800536101020 * filterResult->v[0]); //Very precise butterworth filter

	filterResult->result = filterResult->value[0] + filterResult->value[1];
}

bool max30100_detect_pulse(max30100_t *obj ,float sensor_value){
	 static float prev_sensor_value = 0;
	 static uint8_t values_went_down = 0;
	 static uint32_t current_beat = 0;
	 static uint32_t last_beat = 0;

	 if(sensor_value > PULSE_MAX_THRESHOLD)
	   {
	     obj->current_pulse_detector_state = PULSE_IDLE;
	     prev_sensor_value = 0;
	     last_beat = 0;
	     current_beat = 0;
	     values_went_down = 0;
	     obj->last_beat_threshold = 0;
	     return false;
	   }

	   switch(obj->current_pulse_detector_state)
	   {
	     case PULSE_IDLE:
	       if(sensor_value >= PULSE_MIN_THRESHOLD) {
	    	 obj->current_pulse_detector_state = PULSE_TRACE_UP;
	         values_went_down = 0;
	       }
	       break;

	     case PULSE_TRACE_UP:
	       if(sensor_value > prev_sensor_value)
	       {
	    	 current_beat = HAL_GetTick();
	    	 obj->last_beat_threshold = sensor_value;
	       }
	       else
	       {

#ifdef _DEBUG_
	printf("Peak reached: %f %f\r\n", sensor_value, prev_sensor_value);
#endif

	         uint32_t beat_duration = current_beat - last_beat;
	         last_beat = current_beat;

	         float raw_BPM = 0;
	         if(beat_duration > 0)
	        	 raw_BPM = 60000.0 / (float)beat_duration;

#ifdef _DEBUG_
	printf("RAW bpm: %f\r\n", raw_BPM);
#endif
	         //This method sometimes glitches, it's better to go through whole moving average everytime
	         //IT's a neat idea to optimize the amount of work for moving avg. but while placing, removing finger it can screw up
	         //valuesBPMSum -= valuesBPM[bpmIndex];
	         //valuesBPM[bpmIndex] = rawBPM;
	         //valuesBPMSum += valuesBPM[bpmIndex];

	         obj->values_bmp[obj->bmp_index] = raw_BPM;
	         obj->values_bmp_sum = 0;
	         for(int i=0; i < PULSE_BPM_SAMPLE_SIZE; i++)
	         {
	           obj->values_bmp_sum += obj->values_bmp[i];
	         }

#ifdef _DEBUG
	float message;         _
	printf("Current Moving Avg:\r\n");
	for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++){
		message = obj->values_bmp[i]
		printf("%f\r\n", message);
	}
#endif

	         obj->bmp_index++;
	         obj->bmp_index = obj->bmp_index % PULSE_BPM_SAMPLE_SIZE;

	         if(obj->values_bmp_count < PULSE_BPM_SAMPLE_SIZE)
	        	 obj->values_bmp_count++;

	         obj->current_bmp = obj->values_bmp_sum / obj->values_bmp_count;

#ifdef _DEBUG
	float message = obj->current_bmp;
	printf("AVg. BMP: %f \r\n", message);
#endif


			obj->current_pulse_detector_state = PULSE_TRACE_DOWN;

	         return true;
	       }
	       break;

	     case PULSE_TRACE_DOWN:
	       if(sensor_value < prev_sensor_value)
	       {
	         values_went_down++;
	       }


	       if(sensor_value < PULSE_MIN_THRESHOLD)
	       {
	    	   obj->current_pulse_detector_state = PULSE_IDLE;
	       }
	       break;
	   }

	   prev_sensor_value = sensor_value;
	   return false;
}

void balanceIntesities(max30100_t *obj, float redLedDC, float IRLedDC ){
	if( HAL_GetTick() - obj->last_RED_led_current_check >= RED_LED_CURRENT_ADJUSTMENT_MS)
	  {
	    //Serial.println( redLedDC - IRLedDC );
	    if( IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && obj->RED_led_current < max30100_red_pa_50) // 15 --- led current 50 mA
	    {
	      obj->RED_led_current++;

	      max30100_led_pa_config( obj, obj->RED_led_current, red_led ); // check for correctly use function
#ifdef _DEBUG
	printf("RED LED Current Up\r\n");
#endif
	    }
	    else if(redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && obj->RED_led_current > 0)
	    {
	    	obj->RED_led_current--;
	    	max30100_led_pa_config( obj, obj->RED_led_current, red_led );
#ifdef _DEBUG
	printf("RED LED Current Down\r\n");
#endif
	    }

	   obj->last_RED_led_current_check = HAL_GetTick();
	  }
}
