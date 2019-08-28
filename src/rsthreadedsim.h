/// rsthreadedsim.h - Definitions for threaded simulation code
/// Marc Brooker, 19 July 2006
/// Edited by Yaaseen Martin, 27 August 2019

#ifndef __RSTHREADEDSIM_H
#define __RSTHREADEDSIM_H

#include "rsworld.h"

namespace rs {
    void RunThreadedSim(int thread_limit, World *world);
}

#endif
