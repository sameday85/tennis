/*
 * Copyright (C) 2018 The Automatic Tennis Ball Collector Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/
#include <unistd.h>
#include <sys/time.h>

#include "utils.h"

using namespace std; 

//@returns the current system time in milliseconds
long Utils::current_time_ms() {
    struct timeval start;
    gettimeofday(&start, NULL);

    return (long)(start.tv_usec / 1000 + start.tv_sec * 1000);
}

//suspend execution for millisecond intervals
void Utils::delay_ms(int x) {
    usleep(x * 1000);
}
