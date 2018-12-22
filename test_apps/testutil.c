// ----------------------------------------------------------------------
// Copyright (c) 2016, The Regents of the University of California All
// rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
// 
//     * Neither the name of The Regents of the University of California
//       nor the names of its contributors may be used to endorse or
//       promote products derived from this software without specific
//       prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REGENTS OF THE
// UNIVERSITY OF CALIFORNIA BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
// TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
// ----------------------------------------------------------------------

#include <pthread.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include "riffa.h"
#define NUM_TESTS 100

struct result_info {
  unsigned int fail;
  int sent;
  int recvd;
};

struct thread_info {    /* Used as argument to thread_start() */
  // please refer to API of fpga_send() and fpga_recv() at http://riffa.ucsd.edu/node/10 or https://github.com/KastnerRG/riffa/blob/master/driver/linux/riffa.c#L84-L111
  fpga_t * fpga;    
  unsigned int no;   
  unsigned int chnl;   
  unsigned int * in_buffer;  
  unsigned int * out_buffer;
  unsigned int in_len;
  unsigned int offset;
  unsigned int last;
  long long timeout;
  struct result_info rinfo;
};

void* fpga_send_recv (void *arg) {
  struct thread_info *tinfo = (struct thread_info *) arg;
  int sent, recvd, fail, i, j;

  tinfo->rinfo.fail = 0;
  tinfo->rinfo.sent = 0;
  tinfo->rinfo.recvd = 0;

  for (i = 0; i < NUM_TESTS; i++)
  {
    fail = 0;

    sent = fpga_send(tinfo->fpga, tinfo->chnl, tinfo->in_buffer, tinfo->in_len, tinfo->offset, tinfo->last, tinfo->timeout);
    if (sent == -1) 
    {
      printf("ioctl error : %d\n", sent);
      return NULL;
    }

    if (sent != 0) {
      // Recv the data
      recvd = fpga_recv(tinfo->fpga, tinfo->chnl, tinfo->out_buffer, tinfo->in_len + 4, tinfo->timeout);
      if (recvd == -1)
      {
        printf("ioctl error : %d\n", recvd);
        return NULL;
      }

      if (recvd != sent + 4)
      {
        printf("[FAIL] FPGA no: %d; sent: %d\n", tinfo->no, sent);
        printf("[FAIL] FPGA no: %d; recvd: %d\n", tinfo->no, recvd);
        fail++;
      }

      if (i == NUM_TESTS -1)
      {
        for (j = recvd - 12; j < recvd; j++)
          printf("FPGA[%d]: recvBuffer[%d]: 0x%08X\n", tinfo->no, j, tinfo->out_buffer[j]);
        printf("\n\n");
      } 
    }

    tinfo->rinfo.sent += sent;
    tinfo->rinfo.recvd += recvd;
    tinfo->rinfo.fail += fail;
  }

  return NULL;
}

int main(int argc, char** argv) {
  fpga_t * fpga_rst;
  fpga_t * fpga_test[4];
  fpga_info_list info;
  int option;
  int i, j;
  int id;
  unsigned int * sendBuffer;
  unsigned int ** recvBuffer;
  GET_TIME_INIT(2);

  if (argc < 2) {
    printf("Usage:\n");
    printf("\ttestutil 0: list FPGA\n");
    printf("\ttestutil 1 <fpga_num>: reset FPGA num <fpga_num>\n");
    printf("\ttestutil 2: FPGA test - Transaction per Sec\n");
    return -1;
  }

  option = atoi(argv[1]);

  if (option == 0) {
    // List FPGA info
    // Populate the fpga_info_list struct
    if (fpga_list(&info) != 0) {
      printf("Error populating fpga_info_list\n");
      return -1;
    }
    printf("Number of devices: %d\n", info.num_fpgas);
    for (i = 0; i < info.num_fpgas; i++) {
      printf("%d: id:%d\n", i, info.id[i]);
      printf("%d: num_chnls:%d\n", i, info.num_chnls[i]);
      printf("%d: name:%s\n", i, info.name[i]);
      printf("%d: vendor id:%04X\n", i, info.vendor_id[i]);
      printf("%d: device id:%04X\n", i, info.device_id[i]);
    }
  }
  else if (option == 1) { // Reset FPGA
    if (argc < 3) {
      printf("Usage: %s %d <fpga id>\n", argv[0], option);
      return -1;
    }

    id = atoi(argv[2]);

    if (id == 4)
    {
      for (i = 0; i < 3; i++)
      {
        // Get the device with id
        fpga_rst = fpga_open(i);
        if (fpga_rst == NULL) {
          printf("Could not get FPGA %d\n", i);
          return -1;
        }

        // Reset
        fpga_reset(fpga_rst);

        // Done with device
        fpga_close(fpga_rst);
      }
    }
    else
    {
      // Get the device with id
      fpga_rst = fpga_open(id);
      if (fpga_rst == NULL) {
        printf("Could not get FPGA %d\n", id);
        return -1;
      }

      // Reset
      fpga_reset(fpga_rst);

      // Done with device
      fpga_close(fpga_rst);
    }
  }
  else if (option == 2) { // Send data, receive data
    if (argc < 3) {
      printf("Usage: %s %d <fpga num>\n", argv[0], option);
      return -1;
    }

    int numWords = 20400; // 1500 Transaction, 48 bytes (12 words) per transaction
                          // Note that FPGA board has 4 x 5120 words block RAM
                          // which used to store Transactions, therefore max of numWords
                          // is around 20000
    //int id = 0;         // PCIe core support multiple FPGA board
                          // Feel free to change the id if the system has more than 1 core
                          // Otherwise keep it 0. You can also implement threading and send/receive
                          // from multiple FPGA board at the same time
    int total_fail = 0;
    double total_time = 0.0;
    int numTrans = 0;

    int numFPGACard = atoi(argv[2]);
    // Open FPGA
    for (i = 0; i < numFPGACard; i++)
    {
      // Get the device with id
      fpga_test[i] = fpga_open(i);
      if (fpga_test == NULL) {
        printf("Could not get FPGA %d\n", id);
        return -1;
      }
    } 

    // Malloc the output array
    sendBuffer = (unsigned int *)malloc(numWords*4);
    if (sendBuffer == NULL) {
      printf("Could not malloc memory for sendBuffer\n");
      for (i = 0; i < numFPGACard; i++)
        fpga_close(fpga_test[i]);
      return -1;
    }

    // Malloc the input arrays
    recvBuffer = (unsigned int **)malloc(4 * sizeof (unsigned int *));
    if (recvBuffer == NULL) {
      printf("Could not malloc memory for recvBuffer\n");
      free(sendBuffer);
      for (i = 0; i < numFPGACard; i++)
        fpga_close(fpga_test[i]);
      return -1;
    }

    for (i = 0; i < numFPGACard; i++)
    {
      recvBuffer[i] = (unsigned int *)malloc(numWords*4 + 4);
      if (recvBuffer[i] == NULL) {
        printf("Could not malloc memory for recvBuffer[%d]\n", i);
        for (j = 0; j < numFPGACard; j++)
          fpga_close(fpga_test[j]);
        return -1;
      }
    }

    // Initialize the data
    for (i = 0; i < numFPGACard; i++)
      for (j = 0; j < numWords; j++)
        recvBuffer[i][j] = 0;

    for (i = 0; i < numWords; i++)
      sendBuffer[i] = i+1;

    // Threads stuff
    int NTH = numFPGACard;
    pthread_t tid[NTH];
    struct thread_info tinfo[NTH];
    struct result_info rinfo[NTH];

    assert(tinfo != NULL); // null check
    assert(rinfo != NULL); // null check

    for (i = 0; i < NTH; i++)
    {
      tinfo[i].fpga = fpga_test[i];
      tinfo[i].no = i;
      tinfo[i].chnl = 0;
      tinfo[i].in_buffer = sendBuffer;
      tinfo[i].out_buffer = recvBuffer[i];
      tinfo[i].in_len = numWords;
      tinfo[i].offset = 0;      // No offset
      tinfo[i].last = 1;        // Tell FPGA not to wait
      tinfo[i].timeout = 1000;  // 1 sec

      pthread_create(&tid[i], NULL, &fpga_send_recv, &tinfo[i]);
    }

    printf("Number of FPGA in this test = %d\n", numFPGACard);
    printf("Number of test iteration = %d\n", NUM_TESTS);

    // Start test
    GET_TIME_VAL(0);
    //for(i = 0; i < NTH; i++) {
      pthread_join(tid[2], NULL);
    //}
    GET_TIME_VAL(1);

    total_time = (TIME_VAL_TO_MS(1) - TIME_VAL_TO_MS(0))/1000.0;
    numTrans = numFPGACard * (numWords / 12) * NUM_TESTS;

    for (i = 0; i < numFPGACard; i++)
      total_fail += tinfo[i].rinfo.fail;

    printf("FAIL = %d\n", total_fail);
    printf("Total time = %f s\n", total_time);
    printf("Total transaction = %d\n", numTrans);
    printf("TPS = %f\n", numTrans / total_time);

    // Done with device
    for (i = 0; i < numFPGACard; i++)
      fpga_close(fpga_test[i]);
  }
  return 0;
}
