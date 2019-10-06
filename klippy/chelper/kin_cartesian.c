// Cartesian kinematics stepper pulse time generation
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // move_get_coord
#include "pyhelper.h" // errorf
#include "trapq.h" // trapq_find_move

static double
cart_stepper_x_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).x;
}

static double
cart_stepper_y_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).y;
}

static double
cart_stepper_z_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).z;
}

struct stepper_kinematics * __visible
cartesian_stepper_alloc(char axis)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (axis == 'x')
        sk->calc_position = cart_stepper_x_calc_position;
    else if (axis == 'y')
        sk->calc_position = cart_stepper_y_calc_position;
    else if (axis == 'z')
        sk->calc_position = cart_stepper_z_calc_position;
    return sk;
}

// XXX

struct cart_res_stepper {
    struct stepper_kinematics sk;
    double spring_factor, half_smooth_time;
};

static double
res_x_calc_position(struct stepper_kinematics *sk, struct move *m
                    , double move_time)
{
    struct cart_res_stepper *cs = container_of(sk, struct cart_res_stepper, sk);
    // Calculate nominal stepper position
    double base_pos = move_get_coord(m, move_time).x;
    if (! cs->half_smooth_time)
        return base_pos;
    // Calculate average velocity of 'smooth_time' window before position
    double start_time = move_time - cs->half_smooth_time;
    struct move *sm = trapq_find_move(&cs->sk, m, &start_time);
    double start_pos = move_get_coord(sm, start_time).x;
    double start_diff = base_pos - start_pos;
    // Calculate average velocity of 'smooth_time' window after position
    double end_time = move_time + cs->half_smooth_time;
    struct move *em = trapq_find_move(&cs->sk, m, &end_time);
    double end_pos = move_get_coord(em, end_time).x;
    double end_diff = end_pos - base_pos;
    // Add "spring factor" based on estimated acceleration
    return base_pos + (end_diff - start_diff) * cs->spring_factor;
}

static double
res_y_calc_position(struct stepper_kinematics *sk, struct move *m
                    , double move_time)
{
    struct cart_res_stepper *cs = container_of(sk, struct cart_res_stepper, sk);
    double base_pos = move_get_coord(m, move_time).y;
    if (! cs->half_smooth_time)
        return base_pos;
    double start_time = move_time - cs->half_smooth_time;
    struct move *sm = trapq_find_move(&cs->sk, m, &start_time);
    double start_pos = move_get_coord(sm, start_time).y;
    double start_diff = base_pos - start_pos;
    double end_time = move_time + cs->half_smooth_time;
    struct move *em = trapq_find_move(&cs->sk, m, &end_time);
    double end_pos = move_get_coord(em, end_time).y;
    double end_diff = end_pos - base_pos;
    return base_pos + (end_diff - start_diff) * cs->spring_factor;
}

void __visible
cart_set_smooth_velocity(struct stepper_kinematics *sk
                         , double half_smooth_time, double spring_factor)
{
    struct cart_res_stepper *cs = container_of(sk, struct cart_res_stepper, sk);
    if (! half_smooth_time) {
        cs->spring_factor = cs->half_smooth_time = 0.;
        return;
    }
    cs->spring_factor = spring_factor;
    cs->half_smooth_time = half_smooth_time;
}

static int
cart_res_flush(struct stepper_kinematics *sk
               , double step_gen_time, double print_time)
{
    struct cart_res_stepper *cs = container_of(sk, struct cart_res_stepper, sk);
    double hst = cs->half_smooth_time, flush_time = print_time - hst;
    if (flush_time < step_gen_time)
        flush_time = step_gen_time;
    return trapq_flush(&cs->sk, flush_time, hst, hst);
}

struct stepper_kinematics * __visible
cartesian_res_stepper_alloc(char axis)
{
    struct cart_res_stepper *cs = malloc(sizeof(*cs));
    memset(cs, 0, sizeof(*cs));
    list_init(&cs->sk.moves);
    if (axis == 'x')
        cs->sk.calc_position = res_x_calc_position;
    else if (axis == 'y')
        cs->sk.calc_position = res_y_calc_position;
    else if (axis == 'z')
        cs->sk.calc_position = cart_stepper_z_calc_position;
    cs->sk.flush = cart_res_flush;
    return &cs->sk;
}
