// Trapezoidal velocity movement queue
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // free
#include <string.h> // memset
#include "compiler.h" // unlikely
#include "itersolve.h" // itersolve_gen_steps_range
#include "list.h" // list_empty
#include "trapq.h" // trapq_find_move

void __visible
trapq_add_move(struct stepper_kinematics *sk, struct move *m)
{
    struct move *nm = move_alloc();
    memcpy(nm, m, sizeof(*nm));
    list_add_tail(&nm->node, &sk->moves);
}

void
trapq_clear(struct stepper_kinematics *sk)
{
    if (!sk->moves.root.prev)
        return; // XXX
    while (!list_empty(&sk->moves)) {
        struct move *m = list_first_entry(&sk->moves, struct move, node);
        list_del(&m->node);
        free(m);
    }
}

struct move *
trapq_find_move(struct stepper_kinematics *sk, struct move *m, double *ptime)
{
    double end_time = *ptime;
    for (;;) {
        if (unlikely(end_time < 0.)) {
            // Check previous move in list
            if (list_is_first(&m->node, &sk->moves)) {
                end_time = 0.;
                break;
            }
            struct move *prev = list_prev_entry(m, node);
            end_time += m->print_time - prev->print_time;
            if (end_time >= prev->move_t) {
                end_time = 0.;
                break;
            }
            m = prev;
        } else if (unlikely(end_time > m->move_t)) {
            // Check next move in list
            if (list_is_last(&m->node, &sk->moves)) {
                end_time = m->move_t;
                break;
            }
            struct move *next = list_next_entry(m, node);
            end_time -= next->print_time - m->print_time;
            if (end_time <= 0.) {
                end_time = m->move_t;
                break;
            }
            m = next;
        } else {
            break;
        }
    }
    *ptime = end_time;
    return m;
}

int
trapq_flush(struct stepper_kinematics *sk, double flush_time
            , double prev_scan, double next_scan)
{
    if (list_empty(&sk->moves))
        return 0;
    struct move *cur = list_first_entry(&sk->moves, struct move, node);
    struct move null_move;
    double last_print_time = sk->last_print_time;
    for (;;) {
        struct move *m = cur;
        double move_print_time = m->print_time;
        double move_end_time = move_print_time + m->move_t;
        if (last_print_time >= move_end_time) {
            double scan_to_time = move_end_time + prev_scan;
            if (!list_is_last(&m->node, &sk->moves)) {
                struct move *next = list_next_entry(m, node);
                if (scan_to_time > next->print_time)
                    scan_to_time = next->print_time;
            }
            if (last_print_time + .000000001 < scan_to_time) {
                // Insert null move
                memset(&null_move, 0, sizeof(null_move));
                null_move.node.prev = &m->node;
                null_move.node.next = m->node.next;
                null_move.print_time = move_end_time;
                null_move.move_t = scan_to_time - move_end_time;
                null_move.cruise_t = null_move.move_t;
                null_move.start_pos = move_get_coord(m, m->move_t);
                m = &null_move;
                move_print_time = move_end_time;
            } else {
                if (list_is_last(&m->node, &sk->moves))
                    break;
                cur = list_next_entry(m, node);
                continue;
            }
        } else if (next_scan
                   && last_print_time + .000000001 < move_print_time) {
            // Insert null move
            double null_print_time = move_print_time - next_scan;
            if (last_print_time > null_print_time)
                null_print_time = last_print_time;
            memset(&null_move, 0, sizeof(null_move));
            null_move.node.prev = m->node.prev;
            null_move.node.next = &m->node;
            null_move.print_time = null_print_time;
            null_move.move_t = move_print_time - null_print_time;
            null_move.cruise_t = null_move.move_t;
            null_move.start_pos = m->start_pos;
            m = &null_move;
            move_print_time = null_print_time;
        }

        double start = 0., end = m->move_t;
        if (last_print_time > move_print_time)
            start = last_print_time - move_print_time;
        if (move_print_time + start >= flush_time)
            break;
        if (move_print_time + end > flush_time)
            end = flush_time - move_print_time;
        int32_t ret = itersolve_gen_steps_range(sk, m, start, end);
        if (ret)
            return ret;
        last_print_time = move_print_time + end;
    }
    sk->last_print_time = last_print_time;

    // Free moves that are no longer needed
    while (!list_empty(&sk->moves)) {
        struct move *m = list_first_entry(&sk->moves, struct move, node);
        if (last_print_time < m->print_time + m->move_t + prev_scan)
            break;
        list_del(&m->node);
        free(m);
    }

    return 0;
}
