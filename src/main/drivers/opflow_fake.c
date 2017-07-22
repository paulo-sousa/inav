/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <platform.h>

#include "build/build_config.h"

#include "common/utils.h"

#include "opflow.h"
#include "opflow_fake.h"

#ifdef USE_OPFLOW_FAKE
static bool fakeOpflowInit(opflowDev_t * dev)
{
    UNUSED(dev);
    return true;
}

static bool fakeOpflowUpdate(opflowDev_t * dev)
{
    dev->rawData.flowRateRaw[0] = 0;
    dev->rawData.flowRateRaw[1] = 0;
    dev->rawData.quality = 100;
    dev->rawData.deltaTime = 1000;  // 1ms
    return true;
}

bool fakeOpflowDetect(opflowDev_t * dev)
{
    dev->init = &fakeOpflowInit;
    dev->update = &fakeOpflowUpdate;

    memset(&dev->rawData, 0, sizeof(opflowData_t));

    return true;
}
#endif
