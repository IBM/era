/*
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ErrorMacros.h"
#include "RuntimeTest.h"
#include "Server.h"

#include "nvdla_os_inf.h"
#include <string>
#include <cstring>
#include <iostream>
#include <cstdlib> // system

TestAppArgs defaultTestAppArgs = TestAppArgs();
//TestAppArgs defaultTestAppArgs = TestAppArgs();

TestAppArgs testAppArgs = defaultTestAppArgs;
TestInfo testInfo;

static NvDlaError testSetup(const TestAppArgs * appArgs, TestInfo * i)
//NvDlaError testSetup(const TestAppArgs* appArgs, TestInfo* i)
{
    NvDlaError e = NvDlaSuccess;

    std::string imagePath = "";
    NvDlaStatType stat;

    // Do input paths exist?
    if (std::strcmp(appArgs->inputName.c_str(), "") != 0) {
        e = NvDlaStat(appArgs->inputPath.c_str(), &stat);
        if (e != NvDlaSuccess)
            ORIGINATE_ERROR_FAIL(NvDlaError_BadParameter, "Input path does not exist: \"%s\"", appArgs->inputPath.c_str());

        imagePath = /* appArgs->inputPath + "/images/" + */appArgs->inputName;
        e = NvDlaStat(imagePath.c_str(), &stat);
        if (e != NvDlaSuccess)
            ORIGINATE_ERROR_FAIL(NvDlaError_BadParameter, "Image path does not exist: \"%s/%s\"", imagePath.c_str());
    }

    return NvDlaSuccess;

fail:
    return e;
}

static NvDlaError launchServer(const TestAppArgs * appArgs)
//NvDlaError launchServer(const TestAppArgs* appArgs)
{
    NvDlaError e = NvDlaSuccess;
    TestInfo testInfo;

    testInfo.dlaServerRunning = false;
    PROPAGATE_ERROR_FAIL(runServer(appArgs, &testInfo));

fail:
    return e;
}

static NvDlaError launchTest(const TestAppArgs * appArgs)
//NvDlaError launchTest(const TestAppArgs* appArgs)
{
    NvDlaError e = NvDlaSuccess;

    testInfo.dlaServerRunning = false;
    //TestInfo testInfo;
    PROPAGATE_ERROR_FAIL(testSetup(appArgs, &testInfo));
    //PROPAGATE_ERROR_FAIL(testSetup(testAppArgs, &testInfo));

    PROPAGATE_ERROR_FAIL(run(appArgs, &testInfo));
    //PROPAGATE_ERROR_FAIL(run(testAppArgs, &testInfo));

fail:
    return e;
}

void runImageonNVDLA(std::string iImage) {
    NvDlaError e = NvDlaError_TestApplicationFailed;
    testAppArgs.inputName = iImage;
    testAppArgs.rawOutputDump = true;
    //PROPAGATE_ERROR_FAIL(runImage(&testAppArgs, &testInfo));
    e = runImage(&testAppArgs, &testInfo);
    if (e != NvDlaSuccess) {
        //return EXIT_FAILURE;
        NvDlaDebugPrintf("Test failed\n");
    }
    /**else
    {
        NvDlaDebugPrintf("Image Processed, Test Passed\n");
    }**/

}

// This function reads the loadable and creates a new runtime context
extern "C" {
    void initNVDLA() {

        NvDlaError e = NvDlaError_TestApplicationFailed;
        //    TestAppArgs testAppArgs = defaultTestAppArgs;
        bool serverMode = false;
        bool inputPathSet = false;
        NVDLA_UNUSED(inputPathSet);

        //testAppArgs.loadableName = "mnist_loadable2.nvdla";
        testAppArgs.loadableName = "hpvm-mod.nvdla";


        // Launch
        e = launchTest(&testAppArgs);

        if (e != NvDlaSuccess) {
            //return EXIT_FAILURE;
            NvDlaDebugPrintf("Runtime creation failed and loadable not read\n");
        }
        else {
            NvDlaDebugPrintf("Loadable read and Runtime creation successful\n");
        }

    }
}


extern "C" {
    void runImageonNVDLAWrapper(char * Image) {
        runImageonNVDLA(std::string(Image));
    }
}



