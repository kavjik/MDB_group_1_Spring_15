#ifndef queue_h
#define queue_h

#include "Location.h"
#define QUEUE_SIZE 8
class Location_queue
{
public:
	Location_queue(){ queue_start = 0; queue_end = 0; actual_size = 0; }
	void enqueue(Location element)
	{
		elements[queue_end] = element;
		queue_end++;
		if (queue_end == QUEUE_SIZE) queue_end = 0; //if we are outside the scope of the array, we need to go back in the ring
		actual_size++;
	}

	Location dequeue(void)
	{
		if (actual_size>0)
		{
			Location out = elements[queue_start++];
			if (queue_start == QUEUE_SIZE) queue_start = 0;
			actual_size--;
			return(out);
		}
		else{

			Location target;
			target.latitude = -200;
			target.longtitude = -200;
			return target; //meant to be giberish
		}

	}


	int actual_size;
private:
	Location elements[QUEUE_SIZE];
	int queue_start;
	int queue_end;


};


#endif