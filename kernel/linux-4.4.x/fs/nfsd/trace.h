#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
/*
 * Copyright (c) 2014 Christoph Hellwig.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM nfsd

#if !defined(_NFSD_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _NFSD_TRACE_H

#include <linux/tracepoint.h>

#ifdef MY_ABC_HERE
#include "export.h"

DECLARE_EVENT_CLASS(syno_nfsd_io_class,
	TP_PROTO(struct svc_rqst *rqstp,
		 loff_t	offset,
		 unsigned long len,
		 s64 latency,
		 char *client_addr_str),
	TP_ARGS(rqstp, offset, len,
		latency, client_addr_str),
	TP_STRUCT__entry(
		__field(u32, xid)
		__field(loff_t, offset)
		__field(unsigned long, len)
		__field(s64, latency)
		__string(client_addr, client_addr_str)
		__field(int, ver)
	),
	TP_fast_assign(
		__entry->xid = be32_to_cpu(rqstp->rq_xid);
		__entry->offset = offset;
		__entry->len = len;
		__entry->latency = latency;
		__assign_str(client_addr, client_addr_str);
		__entry->ver = rqstp->rq_vers;
	),
	TP_printk("xid=0x%08x offset=%lld len=%lu latency=%lld client=[%s] ver=%d",
		  __entry->xid, __entry->offset, __entry->len,
		  __entry->latency, __get_str(client_addr), __entry->ver)
)

#define DEFINE_SYNO_NFSD_IO_EVENT(name)			\
DEFINE_EVENT(syno_nfsd_io_class, syno_nfsd_##name,	\
	TP_PROTO(struct svc_rqst *rqstp,	\
		 loff_t		offset,		\
		 unsigned long	len,		\
		 s64	latency,		\
		 char   *client_addr_str),	\
	TP_ARGS(rqstp, offset, len, latency, client_addr_str))

DEFINE_SYNO_NFSD_IO_EVENT(read_io_done);
DEFINE_SYNO_NFSD_IO_EVENT(write_io_done);
#endif /* MY_ABC_HERE */

#include "state.h"

DECLARE_EVENT_CLASS(nfsd_stateid_class,
	TP_PROTO(stateid_t *stp),
	TP_ARGS(stp),
	TP_STRUCT__entry(
		__field(u32, cl_boot)
		__field(u32, cl_id)
		__field(u32, si_id)
		__field(u32, si_generation)
	),
	TP_fast_assign(
		__entry->cl_boot = stp->si_opaque.so_clid.cl_boot;
		__entry->cl_id = stp->si_opaque.so_clid.cl_id;
		__entry->si_id = stp->si_opaque.so_id;
		__entry->si_generation = stp->si_generation;
	),
	TP_printk("client %08x:%08x stateid %08x:%08x",
		__entry->cl_boot,
		__entry->cl_id,
		__entry->si_id,
		__entry->si_generation)
)

#define DEFINE_STATEID_EVENT(name) \
DEFINE_EVENT(nfsd_stateid_class, name, \
	TP_PROTO(stateid_t *stp), \
	TP_ARGS(stp))
DEFINE_STATEID_EVENT(layoutstate_alloc);
DEFINE_STATEID_EVENT(layoutstate_unhash);
DEFINE_STATEID_EVENT(layoutstate_free);
DEFINE_STATEID_EVENT(layout_get_lookup_fail);
DEFINE_STATEID_EVENT(layout_commit_lookup_fail);
DEFINE_STATEID_EVENT(layout_return_lookup_fail);
DEFINE_STATEID_EVENT(layout_recall);
DEFINE_STATEID_EVENT(layout_recall_done);
DEFINE_STATEID_EVENT(layout_recall_fail);
DEFINE_STATEID_EVENT(layout_recall_release);

#ifdef MY_ABC_HERE
#include "xdr4.h"

TRACE_EVENT(syno_nfsd4_dispatch,
	TP_PROTO(struct svc_rqst *rqstp, struct nfsd4_compound_state *cstate, struct nfsd4_op *op),
	TP_ARGS(rqstp, cstate, op),
	TP_STRUCT__entry(
		__field(int, opnum)
	),
	TP_fast_assign(
		__entry->opnum = op->opnum;
	),
	TP_printk("dispatch %d",
		__entry->opnum)
);

#include "xdr3.h"

TRACE_EVENT(syno_nfsd_dispatch,
	TP_PROTO(struct svc_rqst *rqstp),
	TP_ARGS(rqstp),
	TP_STRUCT__entry(
		__field(u32, rq_proc)
	),
	TP_fast_assign(
		__entry->rq_proc = rqstp->rq_proc;
	),
	TP_printk("dispatch %u",
		__entry->rq_proc)
);
#endif /* MY_ABC_HERE */

#endif /* _NFSD_TRACE_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trace
#include <trace/define_trace.h>
