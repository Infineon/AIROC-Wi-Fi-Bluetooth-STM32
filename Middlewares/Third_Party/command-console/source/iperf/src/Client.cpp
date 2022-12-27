/*
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/

/*---------------------------------------------------------------
 * Copyright (c) 1999,2000,2001,2002,2003
 * The Board of Trustees of the University of Illinois
 * All Rights Reserved.
 *---------------------------------------------------------------
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software (Iperf) and associated
 * documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 *
 * Redistributions of source code must retain the above
 * copyright notice, this list of conditions and
 * the following disclaimers.
 *
 *
 * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimers in the documentation and/or other materials
 * provided with the distribution.
 *
 *
 * Neither the names of the University of Illinois, NCSA,
 * nor the names of its contributors may be used to endorse
 * or promote products derived from this Software without
 * specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE CONTIBUTORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * ________________________________________________________________
 * National Laboratory for Applied Network Research
 * National Center for Supercomputing Applications
 * University of Illinois at Urbana-Champaign
 * http://www.ncsa.uiuc.edu
 * ________________________________________________________________
 *
 * Client.cpp
 * by Mark Gates <mgates@nlanr.net>
 * -------------------------------------------------------------------
 * A client thread initiates a connect to the server and handles
 * sending and receiving data, then closes the socket.
 * ------------------------------------------------------------------- */

#include <time.h>
#include "headers.h"
#include "Client.hpp"
/* IPERF_MODIFIED Start */
/* As build system looks for all the header files with the name and adding include paths. It includes mutex.h present in some other component
 * Hence files are renamed by appending iperf.
 */
#include "iperf_locale.h"
#include "iperf_thread.h"
#include "iperf_util.h"
#include "iperf_version.h"
#include "Reporter.h"
#include "iperf_sockets.h"
/* IPERF_MODIFIED End */
#include "SocketAddr.h"
#include "PerfSocket.hpp"
#include "Extractor.h"
#include "delay.h"
#include "isochronous.hpp"
#include "pdfs.h"

// const double kSecs_to_usecs = 1e6;
const double kSecs_to_nsecs = 1e9;
const int    kBytes_to_Bits = 8;

#define VARYLOAD_PERIOD 0.1 // recompute the variable load every n seconds
#define MAXUDPBUF 1470

#ifndef INITIAL_PACKETID
# define INITIAL_PACKETID 0
#endif

Client::Client( thread_Settings *inSettings ) {
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | IPERF_DBG_TRACE | IPERF_DBG_STATE, ( "Creating new client.\n" ) );
    /* IPERF_MODIFIED End */

    mSettings = inSettings;
    mBuf = NULL;
    double ct = -1.0;

    if (isCompat(inSettings) && isPeerVerDetect(inSettings)) {
	fprintf(stderr, "%s", warn_compat_and_peer_exchange);
	unsetPeerVerDetect(inSettings);
    }
    if (isUDP(inSettings) && !isCompat(inSettings)) {
	if ((isPeerVerDetect(inSettings) || (inSettings->mMode != kTest_Normal)) && (inSettings->mBufLen < SIZEOF_UDPHDRMSG)) {
	    mSettings->mBufLen = SIZEOF_UDPHDRMSG;
	    fprintf( stderr, warn_buffer_too_small, "Client", mSettings->mBufLen);
	} else if (mSettings->mBufLen < (int) sizeof( UDP_datagram ) ) {
	    mSettings->mBufLen = sizeof( UDP_datagram );
	    fprintf( stderr, warn_buffer_too_small, "Client", mSettings->mBufLen );
	}
    } else {
	if ((isPeerVerDetect(inSettings) || (inSettings->mMode != kTest_Normal)) && (inSettings->mBufLen < SIZEOF_TCPHDRMSG)) {
	    mSettings->mBufLen = SIZEOF_TCPHDRMSG;
	    fprintf( stderr, warn_buffer_too_small, "Client", mSettings->mBufLen);
	}
    }
    // initialize buffer
    /* IPERF_MODIFIED Start */
    mBuf = (char*) malloc(((mSettings->mBufLen > MAXUDPBUF) ? mSettings->mBufLen : MAXUDPBUF));
    /* IPERF_MODIFIED End */
    FAIL_errno( mBuf == NULL, "No memory for buffer\n", mSettings );
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( MEMALLOC_DEBUG | IPERF_DBG_TRACE, IPERF_MEMALLOC_MSG( mBuf, mSettings->mBufLen ) );
    IPERF_DEBUGF( CLIENT_DEBUG | IPERF_DBG_TRACE | IPERF_DBG_STATE, ( "Filling client buffer with data.\n" ) );
    /* IPERF_MODIFIED End */
    pattern( mBuf, ((mSettings->mBufLen > MAXUDPBUF) ? mSettings->mBufLen : MAXUDPBUF));
/* IPERF_MODIFIED Start */
#ifndef NO_FILE_IO
/* IPERF_MODIFIED End */
    if ( isFileInput( mSettings ) ) {
        if ( !isSTDIN( mSettings ) )
            Extractor_Initialize( mSettings->mFileName, mSettings->mBufLen, mSettings );
        else
            Extractor_InitializeFile( stdin, mSettings->mBufLen, mSettings );

        if ( !Extractor_canRead( mSettings ) ) {
            /* IPERF_MODIFIED Start */
            fprintf(stderr, "Cannot read extractor... unsetting file input.\n");
            /* IPERF_MODFIED End */
            unsetFileInput( mSettings );
        }
    }
/* IPERF_MODIFIED Start */
#endif /* NO_FILE_IO */
/* IPERF_MODIFIED End */
#ifdef HAVE_ISOCHRONOUS
    if (isIsochronous(mSettings) && isUDP(mSettings))
	FAIL_errno( !(mSettings->mFPS > 0.0), "Invalid value for frames per second in the isochronous settings\n", mSettings );
#endif

#ifdef HAVE_CLOCK_NANOSLEEP
#ifdef HAVE_CLOCK_GETTIME
    if (isTxStartTime(inSettings)) {
	int rc = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &inSettings->txstart, NULL);
        if (rc) {
	    fprintf(stderr, "failed clock_nanosleep()=%d\n", rc);
	} else {
	    // Mark the epoch start time before the bind call
	    now.setnow();
	    mSettings->txstart_epoch.tv_sec = now.getSecs();
	    mSettings->txstart_epoch.tv_usec = now.getUsecs();
	}
    }
#endif
#endif

    ct = Connect( );

    if ( isReport( inSettings ) ) {
        /* IPERF_MODIFIED Start */
        IPERF_DEBUGF( CLIENT_DEBUG | IPERF_DBG_TRACE | IPERF_DBG_STATE, ( "Client is reporting settings.\n" ) );
        /* IPERF_MODIFIED End */
        ReportSettings( inSettings );
        if ( mSettings->multihdr && isMultipleReport( inSettings ) ) {
            mSettings->multihdr->report->connection.peer = mSettings->peer;
            mSettings->multihdr->report->connection.size_peer = mSettings->size_peer;
            mSettings->multihdr->report->connection.local = mSettings->local;
            SockAddr_setPortAny( &mSettings->multihdr->report->connection.local );
            mSettings->multihdr->report->connection.size_local = mSettings->size_local;
        }
    }

    // InitDataReport handles Barrier for multiple Streams
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | IPERF_DBG_TRACE, ( "Client is initializing a report.\n" ) );
    /* IPERF_MODIFIED End */
    InitReport(mSettings);
    if (mSettings->reporthdr) {
	mSettings->reporthdr->report.connection.connecttime = ct;
    }

    /* IPERF_MODIFIED Start */
    reportstruct = (ReportStruct*) malloc( sizeof( ReportStruct ) );
    memset(reportstruct, 0, sizeof(ReportStruct));
    /* IPERF_MODIFIED End */
    FAIL_errno( reportstruct == NULL, "No memory for report structure\n", mSettings );
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( MEMALLOC_DEBUG | IPERF_DBG_TRACE, IPERF_MEMALLOC_MSG( reportstruct, sizeof( ReportStruct ) ) );
    /* IPERF_MODIFIED End */
    reportstruct->packetID = (isPeerVerDetect(mSettings)) ? 1 : INITIAL_PACKETID;
    reportstruct->errwrite=WriteNoErr;
    reportstruct->emptyreport=0;
    reportstruct->socket = mSettings->mSock;
    /* IPERF_MODIFIED Start */
    readAt = NULL;
    delay_lower_bounds = 0;
    totLen = 0;
    /* IPERF_MODIFIED End */

} // end Client

/* -------------------------------------------------------------------
 * Destructor
 * ------------------------------------------------------------------- */
Client::~Client() {
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | IPERF_DBG_TRACE | IPERF_DBG_STATE, ( "Destroying Client.\n" ) );
    /* IPERF_MODIFIED End */
    if ( mSettings->mSock != INVALID_SOCKET ) {
        /* IPERF_MODIFIED Start */
        int rc = iperf_close( mSettings->mSock );
        /* IPERF_MODIFIED End */
        WARN_errno( rc == SOCKET_ERROR, "close" );
        mSettings->mSock = INVALID_SOCKET;
    }
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( MEMFREE_DEBUG | IPERF_DBG_TRACE, IPERF_MEMFREE_MSG( mBuf ) );
    FREE_PTR( mBuf );
    FREE_PTR(reportstruct);
    /* IPERF_MODIFIED End */
} // end ~Client


/* -------------------------------------------------------------------
 * Setup a socket connected to a server.
 * If inLocalhost is not null, bind to that address, specifying
 * which outgoing interface to use.
 * ------------------------------------------------------------------- */
double Client::Connect( ) {
    int rc;
    double connecttime = -1.0;

    SockAddr_remoteAddr( mSettings );

    assert( mSettings->mHost != NULL );
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | SOCKET_DEBUG | IPERF_DBG_TRACE | IPERF_DBG_STATE, ( "Client is setting up a socket connected to server %s.\n", mSettings->mHost ) );
    /* IPERF_MODIFIED End */

    // create an internet socket
    int type = ( isUDP( mSettings )  ?  SOCK_DGRAM : SOCK_STREAM);

    /* IPERF_MODIFIED Start */
    /*
    * INTENTIONAL: CID 29413: Identical code for different branches (IDENTICAL_BRANCHES)
    * Reason:
    * This is expected if HAVE_IPV6 is not enabled at runtime. This is not a bug and is the intended behaviour
    * and hence can be ignored.
    */
    /* IPERF_MODIFIED End */
    int domain = (SockAddr_isIPv6( &mSettings->peer ) ?
#ifdef HAVE_IPV6
                  AF_INET6
#else
                  AF_INET
#endif
                  : AF_INET);
    /* IPERF_MODIFIED Start */
    mSettings->mSock = iperf_socket( domain, type, 0 );
    /* IPERF_MODIFIED End */
    WARN_errno( mSettings->mSock == INVALID_SOCKET, "socket" );

    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( SOCKET_DEBUG | IPERF_DBG_TRACE, ("Client is setting socket options for socket %d: {\n"
            "\tTCP window = %d\n"
            "\tCongestion control = %s\n"
            "\tMulticast = %s\n"
            "\tMulticast TTL = %d\n"
            "\tIP TOS = %d\n"
            "\tTCP MSS = %d\n"
    		"\tTCP no delay = %s\n}\n",
    		mSettings->mSock,
    		mSettings->mTCPWin,
    		isCongestionControl( mSettings ) ? "yes" : "no",
    		isMulticast( mSettings ) ? "yes" : "no",
    		mSettings->mTTL,
    		mSettings->mTOS,
    		mSettings->mMSS,
    		isNoDelay( mSettings ) ? "yes" : "no" ) );
    /* IPERF_MODIFIED End */

    SetSocketOptions( mSettings );

    SockAddr_localAddr( mSettings );

    if ( mSettings->mLocalhost != NULL ) {
        // bind socket to local address
        /* IPERF_MODIFIED Start */
        IPERF_DEBUGF( CLIENT_DEBUG | SOCKET_DEBUG | IPERF_DBG_TRACE, ( "Client is binding socket %d to localhost %s.\n", mSettings->mSock, mSettings->mLocalhost ) );
        rc = iperf_bind( mSettings->mSock, (sockaddr*) &mSettings->local,
                   SockAddr_get_sizeof_sockaddr( &mSettings->local ) );
        /* IPERF_MODIFIED End */
        WARN_errno( rc == SOCKET_ERROR, "bind" );
    }

    // Bound the TCP connect() to the -t value (if it was given on the command line)
    // otherwise let TCP use its defaul timeouts fo the connect()
    if (isModeTime(mSettings) && !isUDP(mSettings)) {
	SetSocketOptionsSendTimeout(mSettings, (mSettings->mAmount * 10000));
    }

    // connect socket
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | SOCKET_DEBUG | IPERF_DBG_TRACE, ( "Client is connecting to %s using socket %d.\n", mSettings->mHost, mSettings->mSock ) );
    /* IPERF_MODIFIED End */
    if (!isUDP(mSettings) && isEnhanced(mSettings)) {
	connect_start.setnow();
    /* IPERF_MODIFIED Start */
    rc = iperf_connect( mSettings->mSock, (sockaddr*) &mSettings->peer,
              SockAddr_get_sizeof_sockaddr( &mSettings->peer ));
    FAIL_errno( rc == SOCKET_ERROR, "connect", mSettings );
    /* IPERF_MODIFIED End */
	connect_done.setnow();
	connecttime = 1e3 * connect_done.subSec(connect_start);
    } else {
    /* IPERF_MODIFIED Start */
    rc = iperf_connect( mSettings->mSock, (sockaddr*) &mSettings->peer,
              SockAddr_get_sizeof_sockaddr( &mSettings->peer ));
    /* IPERF_MODIFIED End */
    }
    FAIL_errno( rc == SOCKET_ERROR, "connect", mSettings );

    /* IPERF_MODIFIED Start */
    iperf_getsockname( mSettings->mSock, (sockaddr*) &mSettings->local,
                 &mSettings->size_local );
    iperf_getpeername( mSettings->mSock, (sockaddr*) &mSettings->peer,
                 &mSettings->size_peer );
    /* IPERF_MODIFIED End */
    SockAddr_Ifrname(mSettings);
    return connecttime;

} // end Connect


/* -------------------------------------------------------------------
 * Common traffic loop intializations
 * ------------------------------------------------------------------- */
void Client::InitTrafficLoop (void) {
    //  Enable socket write timeouts for responsive reporting
    //  Do this after the connection establishment
    //  and after Client::InitiateServer as during these
    //  default socket timeouts are preferred.
    int sosndtimer = 0;
    // sosndtimer units microseconds
    if (mSettings->mInterval) {
	sosndtimer = (int) (mSettings->mInterval * 1000000) / 2;
    } else if (isModeTime(mSettings)) {
	sosndtimer = (mSettings->mAmount * 10000) / 2;
    }
    SetSocketOptionsSendTimeout(mSettings, sosndtimer);
    // set the lower bounds delay based of the socket timeout timer
    // units needs to be in nanoseconds
    delay_lower_bounds = (double) sosndtimer * -1e3;

    // set the total bytes sent to zero
    totLen = 0;

    /*
     * Set up common termination variables
     *
     * Terminate the thread by setitimer's alarm (if possible)
     * as the alarm will break a blocked syscall (i.e. the write)
     * and provide for accurate timing. Otherwise the thread cannot
     * terminate until the write completes or the socket SO_SNDTIMEO occurs.
     *
     * In the case of no setitimer we're just using the gettimeofday (or equivalent)
     * calls to determine if the loop time exceeds the request time
     * and the blocking writes will affect timing.  The socket has set
     * SO_SNDTIMEO to 1/2 the overall time (which should help limit
     * gross error) or 1/2 the report interval time (better precision)
     *
     * Side note: An advantage of not using interval reports w/TCP is that
     * the code path won't make any clock syscalls in the main loop
     *
     * For Dual and TradeOff tests we can't use itimer in the Client
     * thread because it is executed at both ends, conflicting with
     * the Server thread's itimer.  The Client process then rejects
     * the reverse connection, and the Server process exits early.  To
     * resolve this, only use the itimer mechanism for "Normal" tests.
     */

    if (isModeTime(mSettings)) {
#ifdef HAVE_SETITIMER
        if (mSettings->mMode == kTest_Normal) {
	    int err;
	    struct itimerval it;
	    memset (&it, 0, sizeof (it));
	    it.it_value.tv_sec = (int) (mSettings->mAmount / 100.0);
	    it.it_value.tv_usec = (int) (10000 * (mSettings->mAmount -
						  it.it_value.tv_sec * 100.0));
	    err = setitimer( ITIMER_REAL, &it, NULL );
	    FAIL_errno( err != 0, "setitimer", mSettings );
	}
#endif
        mEndTime.setnow();
        mEndTime.add( mSettings->mAmount / 100.0 );

        /* IPERF_MODIFIED Start */
        IPERF_DEBUGF( CLIENT_DEBUG | TIME_DEBUG | IPERF_DBG_TRACE, ( "Client set end time to %ld sec %ld usec.\n", mEndTime.getSecs( ), mEndTime.getUsecs( ) ) );
        /* IPERF_MODIFIED End */
    }

    lastPacketTime.setnow();
/* IPERF_MODIFIED Start */
#ifndef NO_FILE_IO
/* IPERF_MODIFIED End */
    readAt = mBuf;
/* IPERF_MODIFIED Start */
#endif
/* IPERF_MODIFIED End */

}


/* -------------------------------------------------------------------
 * Run the appropriate send loop between
 *
 * 1) TCP without rate limiting
 * 2) TCP with rate limiting
 * 3) UDP
 * 4) UDP isochronous w/vbr
 *
 * ------------------------------------------------------------------- */
void Client::Run( void ) {

    // Post the very first report which will have connection, version and test information
    PostFirstReport(mSettings);
    // Peform common traffic setup
    InitTrafficLoop();
    /*
     * UDP specific setup
     */
    if (isUDP(mSettings)) {
	// Preset any UDP fields in the mBuf, a non-zero
	// return indicates some udptests were set
	int udptests = Settings_GenerateClientHdr(mSettings, (client_hdr *) (mBuf + sizeof(struct UDP_datagram)));

/* IPERF_MODIFIED Start */
    (void)udptests;
#ifndef NO_FILE_IO
/* IPERF_MODIFIED End */
	if ( isFileInput( mSettings ) ) {
	    // Due to the UDP timestamps etc, included
	    // reduce the read size by an amount
	    // equal to the header size
	    if ( isCompat( mSettings ) ) {
		Extractor_reduceReadSize( sizeof(struct UDP_datagram), mSettings );
		readAt += sizeof(struct UDP_datagram);
	    } else {
		if (udptests) {
		    Extractor_reduceReadSize(sizeof(client_hdr_udp_tests), mSettings );
		    readAt += sizeof(client_hdr_udp_tests);
		} else {
		    Extractor_reduceReadSize( sizeof(struct UDP_datagram) +
					      sizeof(struct client_hdr), mSettings );
		    readAt += sizeof(struct UDP_datagram) + sizeof(struct client_hdr);
		}
	    }
	}
/* IPERF_MODIFIED Start */
#endif
/* IPERF_MODIFIED End */

	// Launch the approprate UDP traffic loop
	if (isIsochronous(mSettings)) {
	    RunUDPIsochronous();
	} else {
	    RunUDP();
	}
    } else {
	// Launch the approprate TCP traffic loop
	if (mSettings->mUDPRate > 0)
	    RunRateLimitedTCP();
	else
	    RunTCP();
    }
}

/*
 * TCP send loop
 */


void Client::RunTCP( void ) {
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | IPERF_DBG_TRACE | IPERF_DBG_STATE, ( "TCP client running.\n" ) );
    /* IPERF_MODIFIED End */

    int currLen = 0;

    while (InProgress()) {
        // perform write
        if (!isModeTime(mSettings)) {
            /* IPERF_MODIFIED Start */
            IPERF_DEBUGF_COUNTER( SOCKET_DEBUG | IPERF_DBG_TRACE, ( "Client is writing %d bytes to socket %d [total length=%lu].\n", (mSettings->mAmount < (unsigned) mSettings->mBufLen) ? mSettings->mAmount : mSettings->mBufLen, mSettings->mSock, (long unsigned) totLen ) );
            currLen = iperf_write( mSettings->mSock, mBuf, (mSettings->mAmount < (unsigned) mSettings->mBufLen) ? mSettings->mAmount : mSettings->mBufLen);
            /* IPERF_MODIFIED End */
	} else {
            /* IPERF_MODIFIED Start */
        IPERF_DEBUGF_COUNTER( SOCKET_DEBUG | IPERF_DBG_TRACE, ( "Client is writing %d bytes to socket %d [total length=%lu].\n", mSettings->mBufLen, mSettings->mSock, (long unsigned) totLen ) );
            currLen = iperf_write( mSettings->mSock, mBuf, mSettings->mBufLen);
            /* IPERF_MODIFIED End */
	}
        if ( currLen < 0 ) {
	    if (NONFATALTCPWRITERR(errno)) {
	        reportstruct->errwrite=WriteErrAccount;
	    } else if (FATALTCPWRITERR(errno)) {
	        reportstruct->errwrite=WriteErrFatal;
	        WARN_errno( 1, "write" );
		break;
	    } else {
	        reportstruct->errwrite=WriteErrNoAccount;
	    }
	    currLen = 0;
	} else {
	    totLen += currLen;
	    reportstruct->errwrite=WriteNoErr;
	}
// skip the packet time setting syscall() for the case of no interval reporting
// or packet reporting needed and an itimer is available to stop the traffic/while loop
#ifdef HAVE_SETITIMER
	if ((mSettings->mInterval > 0) || isEnhanced(mSettings) ||
	    mSettings->mMode != kTest_Normal)
#endif
	{
	    now.setnow();
	    reportstruct->packetTime.tv_sec = now.getSecs();
	    reportstruct->packetTime.tv_usec = now.getUsecs();
	}

	if ((mSettings->mInterval > 0) || isEnhanced(mSettings)) {
            reportstruct->packetLen = currLen;
            ReportPacket( mSettings->reporthdr, reportstruct );
        }

        if (!isModeTime(mSettings)) {
            /* mAmount may be unsigned, so don't let it underflow! */
            if( mSettings->mAmount >= (unsigned long) currLen ) {
                mSettings->mAmount -= (unsigned long) currLen;
            } else {
                mSettings->mAmount = 0;
            }
        }
    }

    FinishTrafficActions();
}

/*
 * A version of the transmit loop that supports TCP rate limiting using a token bucket
 */
void Client::RunRateLimitedTCP ( void ) {
    int currLen = 0;
    double tokens = 0;
    Timestamp time1, time2;

    int var_rate = mSettings->mUDPRate;
    int fatalwrite_err = 0;
    while (InProgress() && !fatalwrite_err) {
	// Add tokens per the loop time
	// clock_gettime is much cheaper than gettimeofday() so
	// use it if possible.
	time2.setnow();
        if (isVaryLoad(mSettings)) {
	    static Timestamp time3;
	    if (time2.subSec(time3) >= VARYLOAD_PERIOD) {
        /* IPERF_MODIFIED_Start */
		var_rate = (int)lognormal(mSettings->mUDPRate,mSettings->mVariance);
        /* IPERF_MODIFIED_End */
		time3 = time2;
		if (var_rate < 0)
		    var_rate = 0;
	    }
	}
	tokens += time2.subSec(time1) * (var_rate / 8.0);
	time1 = time2;
	if (tokens >= 0.0) {
	    // perform write
	    if (!isModeTime(mSettings)) {
            /* IPERF_MODIFIED Start */
            currLen = iperf_write( mSettings->mSock, mBuf, (mSettings->mAmount < (unsigned) mSettings->mBufLen) ? mSettings->mAmount : mSettings->mBufLen);
            /* IPERF_MODIFIED End */
	    } else {
            /* IPERF_MODIFIED Start */
            currLen = iperf_write( mSettings->mSock, mBuf, mSettings->mBufLen);
            /* IPERF_MODIFIED End */
	    }
	    if ( currLen < 0 ) {
	        if (NONFATALTCPWRITERR(errno)) {
		    reportstruct->errwrite=WriteErrAccount;
		} else if (FATALTCPWRITERR(errno)) {
		    reportstruct->errwrite=WriteErrFatal;
		    WARN_errno( 1, "write" );
		    fatalwrite_err = 1;
		    break;
		} else {
		    reportstruct->errwrite=WriteErrNoAccount;
	        }
	        currLen = 0;
	    } else {
	      // Consume tokens per the transmit
	        tokens -= currLen;
	        totLen += currLen;
		reportstruct->errwrite=WriteNoErr;
	    }
	    time2.setnow();
	    reportstruct->packetTime.tv_sec = time2.getSecs();
	    reportstruct->packetTime.tv_usec = time2.getUsecs();

	    if (isEnhanced(mSettings) || (mSettings->mInterval > 0)) {
		reportstruct->packetLen = currLen;
		ReportPacket( mSettings->reporthdr, reportstruct );
	    }

	    if (!isModeTime(mSettings)) {
		/* mAmount may be unsigned, so don't let it underflow! */
		if( mSettings->mAmount >= (unsigned long) currLen ) {
		    mSettings->mAmount -= (unsigned long) currLen;
		} else {
		    mSettings->mAmount = 0;
		}
	    }
        } else {
	    // Use a 4 usec delay to fill tokens
	    delay_loop(4);
	}
    }

    FinishTrafficActions();
}

/*
 * UDP send loop
 */
void Client::RunUDP( void ) {
    struct UDP_datagram* mBuf_UDP = (struct UDP_datagram*) mBuf;
    int currLen;

    double delay_target = 0;
    double delay = 0;
    double adjust = 0;

    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | IPERF_DBG_TRACE | IPERF_DBG_STATE, ( "Running Client.\n" ) );
    /* IPERF_MODIFIED End */

    // compute delay target in units of nanoseconds
    if (mSettings->mUDPRateUnits == kRate_BW) {
	// compute delay for bandwidth restriction, constrained to [0,1] seconds
	delay_target = (double) ( mSettings->mBufLen * ((kSecs_to_nsecs * kBytes_to_Bits)
							/ mSettings->mUDPRate) );
    } else {
	delay_target = 1e9 / mSettings->mUDPRate;
    }

    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | TIME_DEBUG | IPERF_DBG_TRACE, ( "Client calculated time delay for bandwidth restriction as %d usec.\n", delay_target ) );
    /* IPERF_MODIFIED End */

    if ( delay_target < 0  ||
	 delay_target > 1.0 * kSecs_to_nsecs ) {
	fprintf( stderr, warn_delay_large, delay_target / kSecs_to_nsecs );
	delay_target = 1.0 * kSecs_to_nsecs;
    }

    // Set this to > 0 so first loop iteration will delay the IPG
    currLen = 1;
    double variance = mSettings->mVariance;

    while (InProgress()) {
        // Test case: drop 17 packets and send 2 out-of-order:
        // sequence 51, 52, 70, 53, 54, 71, 72
        //switch( datagramID ) {
        //  case 53: datagramID = 70; break;
        //  case 71: datagramID = 53; break;
        //  case 55: datagramID = 71; break;
        //  default: break;
        //}
	now.setnow();
	reportstruct->packetTime.tv_sec = now.getSecs();
	reportstruct->packetTime.tv_usec = now.getUsecs();
        if (isVaryLoad(mSettings) && mSettings->mUDPRateUnits == kRate_BW) {
	    static Timestamp time3;
	    if (now.subSec(time3) >= VARYLOAD_PERIOD) {
        /* IPERF_MODIFIED_Start */
		int var_rate = (int)lognormal(mSettings->mUDPRate,variance);
        /* IPERF_MODIFIED_End */
		if (var_rate < 0)
		    var_rate = 0;

        /* IPERF_MODIFIED Start */
        if(var_rate != 0) {
        /* IPERF_MODIFIED End */
		delay_target = (double) ( mSettings->mBufLen * ((kSecs_to_nsecs * kBytes_to_Bits)
								/ var_rate) );
        /* IPERF_MODIFIED Start */
        }
        /* IPERF_MODIFIED End */
		time3 = now;
	    }
	}
	// store datagram ID into buffer
	WritePacketID(reportstruct->packetID++);
	mBuf_UDP->tv_sec  = htonl(reportstruct->packetTime.tv_sec);
	mBuf_UDP->tv_usec = htonl(reportstruct->packetTime.tv_usec);

	// Adjustment for the running delay
	// o measure how long the last loop iteration took
	// o calculate the delay adjust
	//   - If write succeeded, adjust = target IPG - the loop time
	//   - If write failed, adjust = the loop time
	// o then adjust the overall running delay
	// Note: adjust units are nanoseconds,
	//       packet timestamps are microseconds
	if (currLen > 0)
	    adjust = delay_target + \
		(1000.0 * lastPacketTime.subUsec( reportstruct->packetTime ));
	else
	    adjust = 1000.0 * lastPacketTime.subUsec( reportstruct->packetTime );

	lastPacketTime.set( reportstruct->packetTime.tv_sec,
			    reportstruct->packetTime.tv_usec );
	// Since linux nanosleep/busyloop can exceed delay
	// there are two possible equilibriums
	//  1)  Try to perserve inter packet gap
	//  2)  Try to perserve requested transmit rate
	// The latter seems preferred, hence use a running delay
	// that spans the life of the thread and constantly adjust.
	// A negative delay means the iperf app is behind.
	delay += adjust;
        /* IPERF_MODIFIED Start */
        IPERF_DEBUGF_COUNTER( TIME_DEBUG | IPERF_DBG_TRACE, ( "Client adjusted delay time by %d usec. Delay time is now %d usec.\n", adjust, delay ) );
        /* IPERF_MODIFIED End */
	// Don't let delay grow unbounded
	if (delay < delay_lower_bounds) {
	    delay = delay_target;
	}

	reportstruct->errwrite = WriteNoErr;
	reportstruct->emptyreport = 0;

	// perform write
        /* IPERF_MODIFIED Start */
        IPERF_DEBUGF_COUNTER( SOCKET_DEBUG | IPERF_DBG_TRACE, ( "Client is writing %d bytes to host %s through socket %d.\n", mSettings->mBufLen, mSettings->mHost, mSettings->mSock ) );
        /* IPERF_MODIFIED End */
        if (!isModeTime(mSettings)) {
        /* IPERF_MODIFIED Start */
            currLen = iperf_write(mSettings->mSock, mBuf, (mSettings->mAmount < (unsigned) mSettings->mBufLen) ? mSettings->mAmount : mSettings->mBufLen);
	} else {
            currLen = iperf_write(mSettings->mSock, mBuf, mSettings->mBufLen);
        /* IPERF_MODIFIED End */
	}
	if ( currLen < 0 ) {
	    reportstruct->packetID--;
	    if (FATALUDPWRITERR(errno)) {
	        reportstruct->errwrite = WriteErrFatal;
	        WARN_errno( 1, "write" );
		break;
	    } else {
	        reportstruct->errwrite = WriteErrAccount;
	        currLen = 0;
	    }
	  reportstruct->emptyreport = 1;
	}

        /* IPERF_MODIFIED Start */
        IPERF_DEBUGF_COUNTER( TIME_DEBUG | IPERF_DBG_TRACE, ( "Packet time for packet %d is %ld sec %ld usec.\n", reportstruct->packetID, (long)reportstruct->packetTime.tv_sec, (long)reportstruct->packetTime.tv_usec ) );
        /* IPERF_MODIFIED End */

	if (!isModeTime(mSettings)) {
	    /* mAmount may be unsigned, so don't let it underflow! */
	    if( mSettings->mAmount >= (unsigned long) currLen ) {
	        mSettings->mAmount -= (unsigned long) currLen;
	    } else {
	        mSettings->mAmount = 0;
	    }
	}

    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF_COUNTER( CLIENT_DEBUG | IPERF_DBG_TRACE, ( "Client is reporting packet %d.\n", reportstruct->packetID ) );
    /* IPERF_MODIFIED End */

	// report packets
	reportstruct->packetLen = (unsigned long) currLen;
	ReportPacket( mSettings->reporthdr, reportstruct );
	// Insert delay here only if the running delay is greater than 1 usec,
	// otherwise don't delay and immediately continue with the next tx.
	if ( delay >= 1000 ) {
	    // Convert from nanoseconds to microseconds
	    // and invoke the microsecond delay
            /* IPERF_MODIFIED Start */
#if HAVE_QUAD_SUPPORT
            IPERF_DEBUGF_COUNTER( CLIENT_DEBUG | TIME_DEBUG | IPERF_DBG_TRACE, ( "Client is delaying for %dms to constrain bandwidth to %llu bits/sec.\n", delay, (long long unsigned) mSettings->mUDPRate ) );
#else
            IPERF_DEBUGF_COUNTER( CLIENT_DEBUG | TIME_DEBUG | IPERF_DBG_TRACE, ( "Client is delaying for %dms to constrain bandwidth to %lu bits/sec.\n", delay, (long unsigned) mSettings->mUDPRate ) );
#endif /* HAVE_QUAD_SUPPORT */

            /* IPERF_MODIFIED End */
	    delay_loop((unsigned long) (delay / 1000));
	}
    }

    FinishTrafficActions();
}

/*
 * UDP isochronous send loop
 */
void Client::RunUDPIsochronous (void) {
#ifndef HAVE_ISOCHRONOUS
    FAIL_errno(1, "UDP isochronous not supported, recompile after using config --enable-isochronous\n", mSettings );
    /* IPERF_MODIFIED Start */
    //return;
	/* IPERF_MODIFIED End */
#else
    struct UDP_datagram* mBuf_UDP = (struct UDP_datagram*) mBuf;
    // skip over the UDP datagram (seq no, timestamp) to reach the isoch fields
    struct client_hdr_udp_isoch_tests *testhdr = (client_hdr_udp_isoch_tests *)(mBuf + sizeof(client_hdr_v1) + sizeof(UDP_datagram));
    struct UDP_isoch_payload* mBuf_isoch = &(testhdr->isoch);

    Isochronous::FrameCounter *fc = new Isochronous::FrameCounter(mSettings->mFPS);

    double delay_target = mSettings->mBurstIPG * 1000000;  // convert from milliseconds to nanoseconds
    double delay = 0;
    double adjust = 0;
    int currLen = 1;
    int frameid=0;
    Timestamp t1;
    int bytecntmin;
    // make sure the packet can carry the isoch payload
    if (isModeTime(mSettings)) {
	bytecntmin = sizeof(UDP_datagram) + sizeof(client_hdr_v1) + sizeof(struct client_hdr_udp_isoch_tests);
    } else {
	bytecntmin = 1;
    }

    mBuf_isoch->burstperiod = htonl(fc->period_us());

    int initdone = 0;
    int fatalwrite_err = 0;
    while (InProgress() && !fatalwrite_err) {
	int bytecnt = (int) (lognormal(mSettings->mMean,mSettings->mVariance)) / (mSettings->mFPS * 8);
	if (bytecnt < bytecntmin)
	    bytecnt = bytecntmin;
	delay = 0;

	// printf("bits=%d\n", (int) (mSettings->mFPS * bytecnt * 8));
	mBuf_isoch->burstsize  = htonl(bytecnt);
	mBuf_isoch->prevframeid  = htonl(frameid);
	reportstruct->burstsize=bytecnt;
	frameid =  fc->wait_tick();
	mBuf_isoch->frameid  = htonl(frameid);
	lastPacketTime.setnow();
	if (!initdone) {
	    initdone = 1;
	    mBuf_isoch->start_tv_sec = htonl(fc->getSecs());
	    mBuf_isoch->start_tv_usec = htonl(fc->getUsecs());
	}

	while ((bytecnt > 0) && InProgress()) {
	    t1.setnow();
	    reportstruct->packetTime.tv_sec = t1.getSecs();
	    reportstruct->packetTime.tv_usec = t1.getUsecs();
	    mBuf_UDP->tv_sec  = htonl(reportstruct->packetTime.tv_sec);
	    mBuf_UDP->tv_usec = htonl(reportstruct->packetTime.tv_usec);
	    WritePacketID(reportstruct->packetID++);

	    // Adjustment for the running delay
	    // o measure how long the last loop iteration took
	    // o calculate the delay adjust
	    //   - If write succeeded, adjust = target IPG - the loop time
	    //   - If write failed, adjust = the loop time
	    // o then adjust the overall running delay
	    // Note: adjust units are nanoseconds,
	    //       packet timestamps are microseconds
	    if (currLen > 0)
		adjust = delay_target + \
		    (1000.0 * lastPacketTime.subUsec( reportstruct->packetTime ));
	    else
		adjust = 1000.0 * lastPacketTime.subUsec( reportstruct->packetTime );

	    lastPacketTime.set( reportstruct->packetTime.tv_sec,
				reportstruct->packetTime.tv_usec );
	    // Since linux nanosleep/busyloop can exceed delay
	    // there are two possible equilibriums
	    //  1)  Try to perserve inter packet gap
	    //  2)  Try to perserve requested transmit rate
	    // The latter seems preferred, hence use a running delay
	    // that spans the life of the thread and constantly adjust.
	    // A negative delay means the iperf app is behind.
	    delay += adjust;
	    // Don't let delay grow unbounded
	    // if (delay < delay_lower_bounds) {
	    //	  delay = delay_target;
	    // }

	    reportstruct->errwrite = WriteNoErr;
	    reportstruct->emptyreport = 0;

	    // perform write
	    if (!isModeTime(mSettings) && (mSettings->mAmount < (unsigned) mSettings->mBufLen)) {
	        mBuf_isoch->remaining = htonl(mSettings->mAmount);
		reportstruct->remaining=mSettings->mAmount;
	        currLen = write(mSettings->mSock, mBuf, mSettings->mAmount);
	    } else {
	        mBuf_isoch->remaining = htonl(bytecnt);
		reportstruct->remaining=bytecnt;
	        currLen = write(mSettings->mSock, mBuf, (bytecnt < mSettings->mBufLen) ? bytecnt : mSettings->mBufLen);
	    }

	    if ( currLen < 0 ) {
	        reportstruct->packetID--;
		reportstruct->emptyreport = 1;
		if (FATALUDPWRITERR(errno)) {
	            reportstruct->errwrite = WriteErrFatal;
	            WARN_errno( 1, "write" );
		    fatalwrite_err = 1;
	        } else {
		    reportstruct->errwrite = WriteErrAccount;
		    currLen = 0;
		}
	    } else {
		bytecnt -= currLen;
		// adjust bytecnt so last packet of burst is greater or equal to min packet
		if ((bytecnt > 0) && (bytecnt < bytecntmin)) {
		    bytecnt = bytecntmin;
		    mBuf_isoch->burstsize  = htonl(bytecnt);
		    reportstruct->burstsize=bytecnt;
		}
	    }

	    if (!isModeTime(mSettings)) {
	        /* mAmount may be unsigned, so don't let it underflow! */
	        if( mSettings->mAmount >= (unsigned long) currLen ) {
		    mSettings->mAmount -= (unsigned long) currLen;
		} else {
		    mSettings->mAmount = 0;
		}
	    }
	    // report packets

	    reportstruct->frameID=frameid;
	    reportstruct->packetLen = (unsigned long) currLen;
	    ReportPacket( mSettings->reporthdr, reportstruct );

	    // Insert delay here only if the running delay is greater than 1 usec,
	    // otherwise don't delay and immediately continue with the next tx.
	    if ( delay >= 1000 ) {
		// Convert from nanoseconds to microseconds
		// and invoke the microsecond delay
		delay_loop((unsigned long) (delay / 1000));
	    }
	}
    }

    FinishTrafficActions();

    DELETE_PTR(fc);
#endif
}
// end RunUDPIsoch



void Client::WritePacketID (intmax_t packetID) {
    struct UDP_datagram * mBuf_UDP = (struct UDP_datagram *) mBuf;
    // store datagram ID into buffer
#ifdef HAVE_INT64_T
    // Pack signed 64bit packetID into unsigned 32bit id1 + unsigned
    // 32bit id2.  A legacy server reading only id1 will still be able
    // to reconstruct a valid signed packet ID number up to 2^31.
    uint32_t id1, id2;
    id1 = packetID & 0xFFFFFFFFLL;
    id2 = (packetID  & 0xFFFFFFFF00000000LL) >> 32;

    mBuf_UDP->id = htonl(id1);
    mBuf_UDP->id2 = htonl(id2);

#ifdef SHOW_PACKETID
    printf("id %" PRIdMAX " (0x%" PRIxMAX ") -> 0x%x, 0x%x\n",
	   packetID, packetID, id1, id2);
#endif
#else
    mBuf_UDP->id = htonl((reportstruct->packetID));
#endif
}

bool Client::InProgress (void) {
/* IPERF_MODIFIED Start */
#ifndef NO_FILE_IO
/* IPERF_MODIFIED End */
    // Read the next data block from
    // the file if it's file input
    if (isFileInput(mSettings)) {
	Extractor_getNextDataBlock( readAt, mSettings );
        if (Extractor_canRead(mSettings) != 0)
	    return true;
	else
	    return false;
    }
/* IPERF_MODIFIED Start */
#endif
/* IPERF_MODIFIED End */


    if (
/* IPERF_MODIFIED Start */
#ifndef NO_INTERRUPTS
sInterupted ||
#endif
/* IPERF_MODIFIED End */
	(isModeTime(mSettings) &&  mEndTime.before(reportstruct->packetTime))  ||
	(!isModeTime(mSettings) && (mSettings->mAmount <= 0)))
	return false;

    return true;
}

/*
 * Common things to do to finish a traffic thread
 */
void Client::FinishTrafficActions(void) {
    // stop timing
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | SOCKET_DEBUG | IPERF_DBG_TRACE | IPERF_DBG_STATE, ( "Client has finished transmitting UDP packets.\n" ) );
    /* IPERF_MODIFIED End */
    now.setnow();
    reportstruct->packetTime.tv_sec = now.getSecs();
    reportstruct->packetTime.tv_usec = now.getUsecs();
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | TIME_DEBUG | IPERF_DBG_TRACE, ( "Client has stopped timing. Stop time is %ld sec %ld us.\n", (long)reportstruct->packetTime.tv_sec, (long)reportstruct->packetTime.tv_usec ) );
    /* IPERF_MODIFIED End */

    /*
     *  For UDP, there is a final handshake between the client and the server,
     *  do that now.
     *
     *  For TCP and if not doing interval or enhanced reporting (needed for write accounting),
     *  then report the entire transfer as one big packet
     *
     */
    if (isUDP(mSettings)) {
	FinalUDPHandshake();
    } else if(!isEnhanced(mSettings) && (0.0 == mSettings->mInterval)) {
	reportstruct->packetLen = totLen;
	ReportPacket( mSettings->reporthdr, reportstruct );
    }

    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | IPERF_DBG_TRACE, ( "Client is closing report with index %d.\n", mSettings->reporthdr->reporterindex ) );
    /* IPERF_MODIFIED End */
    CloseReport( mSettings->reporthdr, reportstruct );
    if (isEnhanced(mSettings) && mSettings->mSock != INVALID_SOCKET ) {
        /* IPERF_MODIFIED Start */
        int rc = iperf_close( mSettings->mSock );
        /* IPERF_MODIFIED End */
        WARN_errno( rc == SOCKET_ERROR, "close" );
        mSettings->mSock = INVALID_SOCKET;
    }
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | IPERF_DBG_TRACE, ( "Client is ending report with index %d.\n", mSettings->reporthdr->reporterindex ) );
    /* IPERF_MODIFIED End */
    EndReport( mSettings->reporthdr );
}


/* -------------------------------------------------------------------
 * Send a datagram on the socket. The datagram's contents should signify
 * a FIN to the application. Keep re-transmitting until an
 * acknowledgement datagram is received.
 * ------------------------------------------------------------------- */
void Client::FinalUDPHandshake(void) {
    struct UDP_datagram * mBuf_UDP = (struct UDP_datagram *) mBuf;
    // send a final terminating datagram
    // Don't count in the mTotalLen. The server counts this one,
    // but didn't count our first datagram, so we're even now.
    // The negative datagram ID signifies termination to the server.

    WritePacketID(-reportstruct->packetID);
    mBuf_UDP->tv_usec = htonl( reportstruct->packetTime.tv_usec );

    if ( isMulticast( mSettings ) ) {
	// Multicast threads only sends one negative sequence number packet
	// and doesn't wait for a server ack
	/* IPERF_MODIFIED Start */
    iperf_write(mSettings->mSock, mBuf, mSettings->mBufLen);
	/* IPERF_MODIFIED End */
    } else {
	// Unicast send and wait for acks
	write_UDP_FIN();
    }
}

void Client::write_UDP_FIN (void) {
    int rc;
    fd_set readSet;
    struct timeval timeout;

    int count = 0;
    while ( count < 10 ) {
        count++;

        // write data
        /* IPERF_MODIFIED Start */
        iperf_write( mSettings->mSock, mBuf, mSettings->mBufLen );
        /* IPERF_MODIFIED End */
	// decrement the packet count
	//
	// Note: a negative packet id is used to tell the server
        // this UDP stream is terminating.  The server will remove
        // the sign.  So a decrement will be seen as increments by
	// the server (e.g, -1000, -1001, -1002 as 1000, 1001, 1002)
        // If the retries weren't decrement here the server can get out
        // of order packets per these retries actually being received
        // by the server (e.g. -1000, -1000, -1000)
	WritePacketID(-(++reportstruct->packetID));

        // wait until the socket is readable, or our timeout expires
        FD_ZERO( &readSet );
        FD_SET( mSettings->mSock, &readSet );
        timeout.tv_sec  = 0;
        timeout.tv_usec = 250000; // quarter second, 250 ms

        /* IPERF_MODIFIED Start */
        rc = iperf_select( mSettings->mSock+1, &readSet, NULL, NULL, &timeout );
        /* IPERF_MODIFIED End */
        FAIL_errno( rc == SOCKET_ERROR, "select", mSettings );

	if ( rc == 0 ) {
            // select timed out
            continue;
        } else {
            // socket ready to read, this packet size
	    // is set by the server.  Assume it's large enough
	    // to contain the final server packet
            /* IPERF_MODIFIED Start */
            rc = iperf_read( mSettings->mSock, mBuf, MAXUDPBUF);
            /* IPERF_MODIFIED End */
	    if ( rc < 0 ) {
                /* IPERF_MODIFIED Start */
                continue;
                /* IPERF_MODIFIED End */
            } else if ( rc >= (int) (sizeof(UDP_datagram) + sizeof(server_hdr)) ) {
                /* IPERF_MODIFIED Start */
                IPERF_DEBUGF( CLIENT_DEBUG | IPERF_DBG_TRACE | IPERF_DBG_STATE, ( "Client will generate a report of the UDP statistics as reported by the server.\n" ) );
                /* IPERF_MODIFIED End */
                ReportServerUDP( mSettings, (server_hdr*) ((UDP_datagram*)mBuf + 1) );
            }
            return;
        }
    }

    fprintf( stderr, warn_no_ack, mSettings->mSock, count );
}
// end write_UDP_FIN


void Client::InitiateServer() {
    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | IPERF_DBG_TRACE | IPERF_DBG_STATE, ( "Client is initiating with the server.\n" ) );
    /* IPERF_MODIFIED End */
    if ( !isCompat( mSettings ) ) {
	int flags = 0;
        client_hdr* temp_hdr;
        if ( isUDP( mSettings ) ) {
            UDP_datagram *UDPhdr = (UDP_datagram *)mBuf;
	    // skip over the UDP datagram (seq no, timestamp)
            temp_hdr = (client_hdr*)(UDPhdr + 1);
        } else {
            temp_hdr = (client_hdr*)mBuf;
        }

    /* IPERF_MODIFIED Start */
    IPERF_DEBUGF( CLIENT_DEBUG | IPERF_DBG_TRACE, ( "Client is generating the client header.\n" ) );
    /* IPERF_MODIFIED End */

	flags = Settings_GenerateClientHdr( mSettings, temp_hdr );

	if (flags & (HEADER_EXTEND | HEADER_VERSION1)) {
	    //  This test requires the pre-test header messages
	    //  The extended headers require an exchange
	    //  between the client and server/listener
	    HdrXchange(flags);
	}
	if (!isUDP(mSettings) && isTripTime(mSettings)) {
        /* IPERF_MODIFIED Start */
        const int inLen = (3 * sizeof(uint32_t));
        /* IPERF_MODIFIED End */
	    char buf[inLen];
	    uint32_t *timers = (uint32_t *) buf;
	    Timestamp t1;
	    *timers++ = htonl(HEADER_TIMESTAMP);
	    *timers++ = htonl(t1.getSecs());
	    *timers++ = htonl(t1.getUsecs());
            /* IPERF_MODIFIED Start */
            IPERF_DEBUGF( SOCKET_DEBUG | IPERF_DBG_TRACE, ( "Client is sending its header to the server using socket %d.\n", mSettings->mSock ) );
            int currLen = iperf_send( mSettings->mSock, buf, inLen, 0 );
            /* IPERF_MODIFIED End */
	    WARN_errno( currLen < 0, "send connect timestamps" );
	}
    }
}


void Client::HdrXchange(int flags) {
    int currLen = 0, len;

    if (flags & HEADER_EXTEND) {
	// Run compatability detection and test info exchange for tests that require it
	int optflag;
	if (isUDP(mSettings)) {
	    struct UDP_datagram* mBuf_UDP = (struct UDP_datagram*) mBuf;
	    Timestamp now;
	    len = mSettings->mBufLen;
	    // UDP header message must be mBufLen so server/Listener will read it
	    // because the Listener read length uses  mBufLen
	    if ((int) (sizeof(UDP_datagram) + sizeof(client_hdr)) > len) {
	        fprintf( stderr, warn_len_too_small_peer_exchange, "Client", len, (sizeof(UDP_datagram) + sizeof(client_hdr)));
	    }
	    // store datagram ID and timestamp into buffer
	    mBuf_UDP->id      = htonl(0);
	    mBuf_UDP->tv_sec  = htonl(now.getSecs());
	    mBuf_UDP->tv_usec = htonl(now.getUsecs());
	} else {
	    len = sizeof(client_hdr);
	    // Disable Nagle to reduce latency of this intial message
	    optflag=1;
            /* IPERF_MODIFIED Start */
            if(iperf_setsockopt( mSettings->mSock, IPPROTO_TCP, TCP_NODELAY, (char *)&optflag, sizeof(int)) < 0 )
            /* IPERF_MODIFIED End */
		WARN_errno(0, "tcpnodelay" );
	}
        /* IPERF_MODIFIED Start */
        currLen = iperf_send( mSettings->mSock, mBuf, len, 0 );
        /* IPERF_MODIFIED End */
	if ( currLen < 0 ) {
	    WARN_errno( currLen < 0, "send_hdr_v2" );
	} else {
	    /* IPERF_MODIFIED Start */
//	    int n;
	    /* IPERF_MODIFIED End */
	    client_hdr_ack ack;
	    int sotimer = 0;
	    // sotimer units microseconds convert
	    if (mSettings->mInterval) {
		sotimer = (int) ((mSettings->mInterval * 1e6) / 4);
	    } else if (isModeTime(mSettings)) {
		sotimer = (int) ((mSettings->mAmount * 1000) / 4);
	    }
	    if (sotimer > HDRXACKMAX) {
		sotimer = HDRXACKMAX;
	    } else if (sotimer < HDRXACKMIN) {
		sotimer = HDRXACKMIN;
	    }
#ifdef WIN32
            // Windows SO_RCVTIMEO uses ms
	    DWORD timeout = (double) sotimer / 1e3;
#else
	    struct timeval timeout;
	    timeout.tv_sec = sotimer / 1000000;
	    timeout.tv_usec = sotimer % 1000000;
#endif
            /* IPERF_MODIFIED Start */
            if (iperf_setsockopt( mSettings->mSock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0 ) {
            /* IPERF_MODIFIED End */
            WARN_errno( mSettings->mSock == SO_RCVTIMEO, "socket" );
	    }
	    /*
	     * Hang a TCP or UDP read and see if this is a header ack message
	     */
		/* IPERF_MODIFIED Start */
	    if (recvn(mSettings->mSock, (char *)&ack, sizeof(client_hdr_ack), 0) == sizeof(client_hdr_ack)) {
		/* IPERF_MODIFIED End */
		if (ntohl(ack.typelen.type) == CLIENTHDRACK && ntohl(ack.typelen.length) == sizeof(client_hdr_ack)) {
		    reporter_peerversion (mSettings, ntohl(ack.version_u), ntohl(ack.version_l));
		} else {
		    sprintf(mSettings->peerversion, " (misformed server version)");
		}
	    } else {
		WARN_errno(1, "recvack" );
		sprintf(mSettings->peerversion, " (server version is old)");
	    }
	}
	if (!isUDP( mSettings ) && !isNoDelay(mSettings)) {
	    optflag = 0;
	    // Re-enable Nagle
            /* IPERF_MODIFIED Start */
            if (iperf_setsockopt( mSettings->mSock, IPPROTO_TCP, TCP_NODELAY, (char *)&optflag, sizeof(int)) < 0 ) {
            /* IPERF_MODIFIED End */
		WARN_errno(0, "tcpnodelay" );
	    }
	}
    } else if (flags & HEADER_VERSION1) {
	if (isUDP(mSettings)) {
	    if ((int) (sizeof(UDP_datagram) + sizeof(client_hdr_v1)) > mSettings->mBufLen) {
		fprintf( stderr, warn_len_too_small_peer_exchange, "Client", mSettings->mBufLen, (sizeof(UDP_datagram) + sizeof(client_hdr_v1)));
	    }
	    // UDP version1 header message is sent as part of normal traffic per Client::Run
	} else {
	    /*
	     * Really should not need this warning as TCP is a byte protocol so the mBufLen shouldn't cause
             * a problem.  Unfortunately, the ver 2.0.5 server didn't read() TCP properly and will fail
             * if the full V1 message does come in a single read.  This was fixed in 2.0.10 but go ahead
	     * and issue a warning in case the server is version 2.0.5
	     */
	    if (((int)sizeof(client_hdr_v1) - mSettings->mBufLen) > 0) {
		fprintf( stderr, warn_len_too_small_peer_exchange, "Client", mSettings->mBufLen, sizeof(client_hdr_v1));
	    }
	    // Send TCP version1 header message now
            /* IPERF_MODIFIED Start */
            currLen = iperf_send( mSettings->mSock, mBuf, sizeof(client_hdr_v1), 0 );
            /* IPERF_MODIFIED End */
	    WARN_errno( currLen < 0, "send_hdr_v1" );
	}
    }
}
