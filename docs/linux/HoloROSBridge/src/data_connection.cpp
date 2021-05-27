//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "data_connection.h"

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void recvMessage(int sock, unsigned int size, char* pointer)
{
    int total=0;
    while (total < size) {
        int n = recv(sock, pointer, (size-total), 0);
        total += n;
        pointer += n;
    }
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
int reverseEndian(int n)
{
    int ret;
    char* n_buf,*ret_buf;

    n_buf = (char*)&n;
    ret_buf = (char*)&ret;
    
    ret_buf[0] = n_buf[3];
    ret_buf[1] = n_buf[2];
    ret_buf[2] = n_buf[1];
    ret_buf[3] = n_buf[0];

    return ret;
}
