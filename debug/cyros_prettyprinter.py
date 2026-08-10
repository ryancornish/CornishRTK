import gdb

def unwrap(val):
    """Strip atomic wrappers until a plain value is left.

    Several shapes reach here: cyros::relaxed_atomic<T> (member 'value'),
    std::atomic<T> over a scalar ('_M_base._M_i'), and std::atomic<T*>
    ('_M_b._M_p'). Reading the storage directly rather than parsing the
    built-in printer's text is what keeps this working for pointers, enums and
    integers alike.
    """
    for _ in range(6):
        try:
            fields = val.type.strip_typedefs().fields()
        except Exception:
            return val

        by_name = {f.name: f for f in fields if f.name}
        stepped = False
        for name in ("value", "_M_base", "_M_i", "_M_b", "_M_p"):
            if name in by_name:
                val = val[name]
                stepped = True
                break
        if stepped:
            continue

        # libstdc++ reaches the storage of an integral std::atomic through a
        # BASE class (__atomic_base<T>), which gdb reports as an unnamed-ish
        # field rather than a member, so descending members alone stops short.
        for field in fields:
            if getattr(field, "is_base_class", False):
                val = val.cast(field.type)
                stepped = True
                break
        if not stepped:
            return val
    return val


def array_length(val):
    """Element count of a gdb array value.

    Derived from the type's index range, NOT from sizeof(array)/sizeof(target):
    these arrays reach gdb through a libstdc++ typedef whose target() is the
    array itself, so the division silently yields 1 and every printer built on
    it shows only the first element.
    """
    low, high = val.type.strip_typedefs().range()
    return high - low + 1


def format_atomic(val):
    """Human-readable value of a possibly-atomic field."""
    try:
        text = str(unwrap(val))
    except Exception:
        return "Unreadable"
    # Fallback for any wrapper unwrap did not recognise: gdb renders those as
    # "std::atomic<bool> = { true }", and only the payload is wanted.
    if "{" in text and "}" in text:
        return text.split("{")[1].split("}")[0].strip()
    return text

def get_tcb_summary(tcb_ptr):
    """Safely dereferences a TCB pointer to grab its ID or returns nullptr/error."""
    try:
        tcb_ptr = unwrap(tcb_ptr)
        addr = int(tcb_ptr)
        if addr == 0:
            return "nullptr"
        # Safely extract the thread ID from the pointed-to TCB structure
        tcb_val = tcb_ptr.dereference()
        t_id = int(tcb_val['id'])
        return f"ID {t_id} (Addr: 0x{addr:x})"
    except Exception:
        return f"0x{int(tcb_ptr):x} [ID unreadable]"

class ThreadControlBlockPrinter:
    def __init__(self, val):
        self.val = val

    # Must match cyros::thread_state. 'created' is the zero value, so an
    # off-by-one here silently renames every state rather than failing.
    STATES = ("created", "ready", "running", "blocked", "terminated")
    DISPOSITIONS = ("none", "prepared", "committed")

    def _enum_string(self, field, names):
        try:
            value = int(unwrap(self.val[field]))
            return names[value] if 0 <= value < len(names) else f"unknown({value})"
        except Exception:
            return "unknown"

    def _held(self):
        """Which PI resources this thread owns, i.e. what its urgency folds over.

        There is no effective_priority to print: urgency is min(base, best
        waiter of every held resource) and is computed at the point of use, so
        the slots below ARE the state that determines it. A boosted holder is
        one with a slot whose queue has a better waiter than its base.
        """
        try:
            mask = int(unwrap(self.val['held_mask']))
        except Exception:
            return "unreadable"
        if mask == 0:
            return "none"
        slots = self.val['held_slots']['_M_elems']
        count = array_length(slots)
        held = []
        for i in range(count):
            if not (mask >> i) & 1:
                continue
            try:
                addr = int(unwrap(slots[i]))
            except Exception:
                addr = 0
            held.append(f"slot{i}=0x{addr:x}" if addr else f"slot{i}=<cleared>")
        return f"mask 0x{mask:x} [{', '.join(held)}]"

    def to_string(self):
        t_id = int(self.val['id'])
        state = self._enum_string('state', self.STATES)
        disposition = self._enum_string('disposition', self.DISPOSITIONS)
        base_pri = int(self.val['base_priority'])
        core = int(self.val['pinned_core'])

        this_addr = self.val.address
        enqueued = "No (Sentinel)" if this_addr == unwrap(self.val['next']) else "Yes"
        listed = "No (Sentinel)" if this_addr == unwrap(self.val['holder_next']) else "Yes"

        return (
            f"TCB [ID: {t_id}] (State: {state}, Disposition: {disposition})\n"
            f"    ├── Base Priority:       {base_pri}\n"
            f"    ├── Holds (PI):          {self._held()}\n"
            f"    ├── Pinned Core:         {core}\n"
            f"    ├── Enqueued Status:     {enqueued}\n"
            f"    └── On Holder List:      {listed}"
        )


class SchedulerPrinter:
    def __init__(self, val):
        self.val = val

    def _holder_chain(self):
        """Ready threads pinned here that hold a PI resource, in pick order.

        These are exactly the threads whose urgency can beat their base
        priority, so this list plus each TCB's held slots is what the pick
        folds. Bounded so a corrupt link prints instead of hanging gdb.
        """
        try:
            node = self.val['holders_head']
        except Exception:
            return "unreadable"
        ids = []
        seen = set()
        node = unwrap(node)
        while int(node) != 0 and len(ids) < 16:
            addr = int(node)
            if addr in seen:
                ids.append("<cycle>")
                break
            seen.add(addr)
            try:
                ids.append(str(int(node.dereference()['id'])))
                node = unwrap(node.dereference()['holder_next'])
            except Exception:
                ids.append(f"<unreadable 0x{addr:x}>")
                break
        return " -> ".join(ids) if ids else "empty"

    def to_string(self):
        core_id = int(self.val['core_id'])
        pinned = format_atomic(self.val['pinned_thread_counter'])

        # The intake head is the whole cross-core request state: the TCB is the
        # message, so a non-null head means work is outstanding for this core.
        intake = get_tcb_summary(self.val['intake_head'])
        holders = self._holder_chain()

        curr_str = get_tcb_summary(self.val['current_thread'])
        idle_str = get_tcb_summary(self.val['idle_thread'])

        try:
            bitmap = int(self.val['ready_matrix']['bitmap'])
            matrix_str = f"0x{bitmap:x} (Raw: {bin(bitmap)})"
        except Exception:
            matrix_str = "Unknown"

        # Explicitly prepend a newline \n to the start of every core block!
        # This completely overrides GDB's attempts to paste them side-by-side.
        return (
            f"\n  Scheduler [Core {core_id}]\n"
            f"    ├── Active TCB:          {curr_str}\n"
            f"    ├── Idle TCB:            {idle_str}\n"
            f"    ├── Pinned Threads:      {pinned}\n"
            f"    ├── Ready Matrix Bitmap: {matrix_str}\n"
            f"    ├── Intake Head:         {intake}\n"
            f"    ├── Holder List:         {holders}\n"
            f"    └── Tie Rotor:           0x{int(self.val['tie_rotor']):x}"
        )


class KernelStatePrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        init = format_atomic(self.val['initialised'])
        run = format_atomic(self.val['running'])
        threads = format_atomic(self.val['active_threads'])
        next_id = format_atomic(self.val['thread_id_generator'])

        return (
            f"KernelState Layout\n"
            f"    ├── Initialised:         {init}\n"
            f"    ├── Running:             {run}\n"
            f"    ├── Active Threads:      {threads}\n"
            f"    └── Next Thread ID Gen:  {next_id}"
        )

    def children(self):
        schedulers = self.val['schedulers']
        try:
            elems = schedulers['_M_elems']
            for i in range(array_length(elems)):
                yield f'Core {i}', elems[i]
        except Exception:
            yield 'schedulers_raw', schedulers


def lookup_cyros_types(val):
    type_tag = val.type.unqualified().strip_typedefs().tag
    if not type_tag:
        return None

    if type_tag.endswith("kernel_state"):
        return KernelStatePrinter(val)
    elif type_tag.endswith("scheduler"):
        return SchedulerPrinter(val)
    elif type_tag.endswith("thread_control_block"):
        return ThreadControlBlockPrinter(val)

    return None

gdb.pretty_printers.append(lookup_cyros_types)
