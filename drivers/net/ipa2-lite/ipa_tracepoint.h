// SPDX-License-Identifier: GPL-2.0-only
//

#undef TRACE_SYSTEM
#define TRACE_SYSTEM ipa2

#define NOTRACE 1

#if !defined(_IPA2_TRACEPOINT_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _IPA2_TRACEPOINT_H_

#include <linux/tracepoint.h>
#include <linux/trace_seq.h>
#include "ipa.h"

TRACE_EVENT(ipa2_event,
	    TP_PROTO(struct ipa_ep *ep, const char *c),
	    TP_ARGS(ep, c),
	    TP_STRUCT__entry(
		__string(name, ep->name)
		__string(c, c)
	    ),
	    TP_fast_assign(
			   __assign_str(name);
			   __assign_str(c);
	    ),

	    TP_printk("%s ep=%s", __get_str(c), __get_str(name))
	    );

TRACE_EVENT(ipa2_fifo_state,
	    TP_PROTO(struct ipa_ep *ep, const char *c),
	    TP_ARGS(ep, c),
	    TP_STRUCT__entry(
		__field(u16, head)
		__field(u16, tail)
		__field(u16, thead)
		__field(u16, ttail)
		__field(u16, hwhead)
		__field(u16, hwtail)
		__string(name, ep->name)
		__string(c, c)
	    ),
	    TP_fast_assign(
			   __entry->head = ep->head;
			   __entry->thead = ep->tx_head;
			   __entry->hwhead = ioread32(ep->hw_head) / 8;
			   __entry->tail = ep->tail;
			   __entry->ttail = ep->tx_tail;
			   __entry->hwtail = ioread32(ep->hw_tail) / 8;
			   __assign_str(name);
			   __assign_str(c);
	    ),

	    TP_printk("%s ep=%s h=%u t=%u TX h=%u t=%u HW h=%u t=%u",
		      __get_str(c), __get_str(name),
		      __entry->head, __entry->tail,
		      __entry->thead, __entry->ttail,
		      __entry->hwhead, __entry->hwtail)
	    );

TRACE_EVENT(ipa2_irq,
	    TP_PROTO(struct ipa_ep *ep, u8 sts, bool dis),
	    TP_ARGS(ep, sts, dis),
	    TP_STRUCT__entry(
		__string(name, ep->name)
		__field(u8, sts)
		__field(bool, dis)
	    ),
	    TP_fast_assign(
			   __entry->sts = sts;
			   __entry->dis = dis;
			   __assign_str(name);
	    ),

	    TP_printk("ep=%s %ssts=%x",
		      __get_str(name),
		      __entry->dis ? "disabled " : "",
		      __entry->sts)
	    );

TRACE_EVENT(ipa2_desc,
	    TP_PROTO(struct ipa_ep *ep, bool write, u16 pos,
		    struct fifo_desc d, const char *c),
	    TP_ARGS(ep, write, pos, d, c),
	    TP_STRUCT__entry(
		__string(c, c)
		__string(name, ep->name)
		__field(bool, write)
		__field(u16, pos)
		__field(u16, size)
		__field(u16, flags)
		__field(u16, addr)
	    ),
	    TP_fast_assign(
			   __assign_str(c);
			   __assign_str(name);
			   __entry->pos = pos;
			   __entry->write = write;
			   __entry->addr = d.addr;
			   __entry->size = d.size;
			   __entry->flags = d.flags;
	    ),

	    TP_printk("%s ep=%s %s pos=%u (addr=%x flags=%x size=%x)",
		      __get_str(c), __get_str(name),
		      __entry->write ? "write" : "read",
		      __entry->pos,
		      __entry->addr, __entry->flags, __entry->size)
	    );

#endif

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE ipa_tracepoint

#include <trace/define_trace.h>
