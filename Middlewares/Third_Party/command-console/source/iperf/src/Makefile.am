bin_PROGRAMS = iperf

LIBCOMPAT_LDADDS = @STRIP_BEGIN@ \
		   $(top_builddir)/compat/libcompat.a \
		   @STRIP_END@

AM_CPPFLAGS = @STRIP_BEGIN@ \
	      -I$(top_srcdir)/include \
	      -I$(top_builddir)/include \
	      @STRIP_END@

#IPERF_MODIFIED Start
if DEBUG
AM_CFLAGS = -Wall -g -O0 -DDEBUG
AM_CXXFLAGS = -Wall -g -O0 -DDEBUG
else
AM_CFLAGS = -Wall -O3
AM_CXXFLAGS = -Wall -O3
endif
#IPERF_MODIFIED End

iperf_LDFLAGS = @CFLAGS@ @PTHREAD_CFLAGS@ @WEB100_CFLAGS@ @DEFS@

iperf_SOURCES = \
		Client.cpp \
		Extractor.c \
	        isochronous.cpp \
		Launch.cpp \
		List.cpp \
		Listener.cpp \
		Locale.c \
		PerfSocket.cpp \
		ReportCSV.c \
		ReportDefault.c \
		Reporter.c \
		Server.cpp \
		Settings.cpp \
		SocketAddr.c \
		gnu_getopt.c \
		gnu_getopt_long.c \
	        histogram.c \
		main.cpp \
		service.c \
		sockets.c \
		stdio.c \
		tcp_window_size.c \
		pdfs.c \
		#IPERF_MODIFIED Start
		debug.c
		#IPERF_MODIFIED End
iperf_LDADD = $(LIBCOMPAT_LDADDS)


if CHECKPROGRAMS
noinst_PROGRAMS = checkdelay checkpdfs checkisoch igmp_querier
checkdelay_SOURCES = checkdelay.c
checkdelay_LDADD = $(LIBCOMPAT_LDADDS)
checkpdfs_SOURCES = pdfs.c checkpdfs.c stdio.c
checkpdfs_LDADD = -lm
checkisoch_SOURCES = checkisoch.cpp isochronous.cpp pdfs.c stdio.c
igmp_querier_SOURCES = igmp_querier.c
checkisoch_LDADD = $(LIBCOMPAT_LDADDS)
endif

if AF_PACKET
iperf_SOURCES += checksums.c
endif

