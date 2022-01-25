struct Clock
{
	u64 performance_frequency;
	float sec_per_count;
	
	u64 last_count;
	u64 end_count;
	
	u64 counter_elapsed;
	
	float t;
};

void init_clock(Clock* clock)
{
	clock->performance_frequency = SDL_GetPerformanceFrequency();
	clock->sec_per_count = 1.0f / (float)clock->performance_frequency;
	clock->last_count = SDL_GetPerformanceCounter();
};

float get_seconds_elapsed(Clock* clock)
{
	float result = (float)((clock->end_count - clock->last_count) * 
							  clock->sec_per_count);
	return result;
}

float get_seconds_elapsed(u64 last_count, u64 current_count, float sec_per_count)
{
	float result = (float)((current_count - last_count) * sec_per_count);
	return result;
}

float get_current_time_in_seconds()
{
	float result = (float)SDL_GetTicks() * ms_in_sec;
	return result;
}

void update_clock(Clock* clock)
{
	clock->t = get_seconds_elapsed(clock);
	clock->counter_elapsed = clock->end_count - clock->last_count;
	clock->last_count = clock->end_count;
}