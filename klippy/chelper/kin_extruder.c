// Extruder stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // trapq_find_move

struct extruder_stepper {
    struct stepper_kinematics sk;
    double pressure_advance_factor, smooth_time;
};

static double
extruder_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    // Calculate nominal extruder position
    double dist = move_get_distance(m, move_time);
    double base_pos = m->start_pos.x + dist;
    if (! es->smooth_time)
        return base_pos;
    // Calculate position 'smooth_time' in the past
    double start_time = move_time - es->smooth_time;
    struct move *sm = trapq_find_move(&es->sk, m, &start_time);
    double start_dist = move_get_distance(sm, start_time);
    double pa_start_pos = sm->start_pos.y + (sm->axes_r.y ? start_dist : 0.);
    // Calculate position 'smooth_time' in the future
    double end_time = move_time + es->smooth_time;
    struct move *em = trapq_find_move(&es->sk, m, &end_time);
    double end_dist = move_get_distance(em, end_time);
    double pa_end_pos = em->start_pos.y + (em->axes_r.y ? end_dist : 0.);
    // Calculate position with pressure advance
    return base_pos + (pa_end_pos - pa_start_pos) * es->pressure_advance_factor;
}

// Populate a 'struct move' with an extruder velocity trapezoid
void __visible
extruder_move_fill(struct stepper_kinematics *sk, double print_time
                   , double accel_t, double cruise_t, double decel_t
                   , double start_pos, double start_pa_pos
                   , double start_v, double cruise_v, double accel
                   , int is_pa_move)
{
    struct move *m = move_alloc();

    // Setup velocity trapezoid
    m->print_time = print_time;
    m->move_t = accel_t + cruise_t + decel_t;
    m->accel_t = accel_t;
    m->cruise_t = cruise_t;
    m->cruise_start_d = accel_t * .5 * (cruise_v + start_v);
    m->decel_start_d = m->cruise_start_d + cruise_t * cruise_v;

    // Setup for accel/cruise/decel phases
    m->cruise_v = cruise_v;
    m->accel.c1 = start_v;
    m->accel.c2 = .5 * accel;
    m->decel.c1 = cruise_v;
    m->decel.c2 = -m->accel.c2;

    // Setup start distance
    m->start_pos.x = start_pos;
    m->start_pos.y = start_pa_pos;
    m->axes_r.x = 1.;
    m->axes_r.y = is_pa_move ? 1. : 0.;

    // Add to list
    list_add_tail(&m->node, &sk->moves);
}

void __visible
extruder_set_pressure(struct stepper_kinematics *sk
                      , double pressure_advance, double smooth_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    if (! smooth_time) {
        es->pressure_advance_factor = es->smooth_time = 0.;
        return;
    }
    es->pressure_advance_factor = pressure_advance / (2. * smooth_time);
    es->smooth_time = smooth_time;
}

// XXX
static int
extruder_flush(struct stepper_kinematics *sk
               , double step_gen_time, double print_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    double flush_time = print_time - es->smooth_time;
    if (flush_time < step_gen_time)
        flush_time = step_gen_time;
    return trapq_flush(&es->sk, flush_time, es->smooth_time, es->smooth_time);
}

// xxx - Need an extruder_stepper_free() function

struct stepper_kinematics * __visible
extruder_stepper_alloc(void)
{
    struct extruder_stepper *es = malloc(sizeof(*es));
    memset(es, 0, sizeof(*es));
    list_init(&es->sk.moves);
    es->sk.calc_position = extruder_calc_position;
    es->sk.flush = extruder_flush;
    return &es->sk;
}
