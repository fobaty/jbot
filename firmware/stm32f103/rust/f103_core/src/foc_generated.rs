//! Bit-exact host-side model of the pinned Simulink-generated EFeru FOC controller.
//!
//! The arithmetic below is a literal transliteration of `BLDC_controller.c`:
//! integer widths, cast order, rounding offsets and the three distinct PI
//! implementations are preserved because the generated lookup tables and
//! anti-windup limits depend on them. Do not "simplify" the expressions: a
//! mathematically equivalent rearrangement changes the fixed-point result.
//!
//! This module is for validation and integration only. The production motor
//! controller stays untouched until Safety-Reviewer approval.

// Parameter and port names deliberately mirror the generated C interface.
#![allow(non_snake_case)]
// The PI helper signatures mirror the generated functions argument for argument.
#![allow(clippy::too_many_arguments)]

// State-machine constants from BLDC_controller.c lines 23-32.
const IN_ACTIVE: u8 = 1;
const IN_NO_ACTIVE_CHILD: u8 = 0;
const IN_OPEN: u8 = 2;
const IN_SPEED_MODE: u8 = 1;
const IN_TORQUE_MODE: u8 = 2;
const IN_VOLTAGE_MODE: u8 = 3;
const OPEN_MODE: u8 = 0;
const SPD_MODE: u8 = 2;
const TRQ_MODE: u8 = 3;
const VLT_MODE: u8 = 1;

const R_SIN_M1_TABLE: [i16; 181] = [
    8192, 8682, 9162, 9630, 10087, 10531, 10963, 11381, 11786, 12176, 12551, 12911, 13255, 13583,
    13894, 14189, 14466, 14726, 14968, 15191, 15396, 15582, 15749, 15897, 16026, 16135, 16225,
    16294, 16344, 16374, 16384, 16374, 16344, 16294, 16225, 16135, 16026, 15897, 15749, 15582,
    15396, 15191, 14968, 14726, 14466, 14189, 13894, 13583, 13255, 12911, 12551, 12176, 11786,
    11381, 10963, 10531, 10087, 9630, 9162, 8682, 8192, 7692, 7182, 6664, 6138, 5604, 5063, 4516,
    3964, 3406, 2845, 2280, 1713, 1143, 572, 0, -572, -1143, -1713, -2280, -2845, -3406, -3964,
    -4516, -5063, -5604, -6138, -6664, -7182, -7692, -8192, -8682, -9162, -9630, -10087, -10531,
    -10963, -11381, -11786, -12176, -12551, -12911, -13255, -13583, -13894, -14189, -14466, -14726,
    -14968, -15191, -15396, -15582, -15749, -15897, -16026, -16135, -16225, -16294, -16344, -16374,
    -16384, -16374, -16344, -16294, -16225, -16135, -16026, -15897, -15749, -15582, -15396, -15191,
    -14968, -14726, -14466, -14189, -13894, -13583, -13255, -12911, -12551, -12176, -11786, -11381,
    -10963, -10531, -10087, -9630, -9162, -8682, -8192, -7692, -7182, -6664, -6138, -5604, -5063,
    -4516, -3964, -3406, -2845, -2280, -1713, -1143, -572, 0, 572, 1143, 1713, 2280, 2845, 3406,
    3964, 4516, 5063, 5604, 6138, 6664, 7182, 7692, 8192,
];

const R_COS_M1_TABLE: [i16; 181] = [
    14189, 13894, 13583, 13255, 12911, 12551, 12176, 11786, 11381, 10963, 10531, 10087, 9630, 9162,
    8682, 8192, 7692, 7182, 6664, 6138, 5604, 5063, 4516, 3964, 3406, 2845, 2280, 1713, 1143, 572,
    0, -572, -1143, -1713, -2280, -2845, -3406, -3964, -4516, -5063, -5604, -6138, -6664, -7182,
    -7692, -8192, -8682, -9162, -9630, -10087, -10531, -10963, -11381, -11786, -12176, -12551,
    -12911, -13255, -13583, -13894, -14189, -14466, -14726, -14968, -15191, -15396, -15582, -15749,
    -15897, -16026, -16135, -16225, -16294, -16344, -16374, -16384, -16374, -16344, -16294, -16225,
    -16135, -16026, -15897, -15749, -15582, -15396, -15191, -14968, -14726, -14466, -14189, -13894,
    -13583, -13255, -12911, -12551, -12176, -11786, -11381, -10963, -10531, -10087, -9630, -9162,
    -8682, -8192, -7692, -7182, -6664, -6138, -5604, -5063, -4516, -3964, -3406, -2845, -2280,
    -1713, -1143, -572, 0, 572, 1143, 1713, 2280, 2845, 3406, 3964, 4516, 5063, 5604, 6138, 6664,
    7182, 7692, 8192, 8682, 9162, 9630, 10087, 10531, 10963, 11381, 11786, 12176, 12551, 12911,
    13255, 13583, 13894, 14189, 14466, 14726, 14968, 15191, 15396, 15582, 15749, 15897, 16026,
    16135, 16225, 16294, 16344, 16374, 16384, 16374, 16344, 16294, 16225, 16135, 16026, 15897,
    15749, 15582, 15396, 15191, 14968, 14726, 14466, 14189,
];

const R_SIN3PHA_A_M1_TABLE: [i16; 181] = [
    -13091, -13634, -14126, -14565, -14953, -15289, -15577, -15816, -16009, -16159, -16269, -16340,
    -16377, -16383, -16362, -16317, -16253, -16172, -16079, -15977, -15870, -15762, -15656, -15555,
    -15461, -15377, -15306, -15248, -15206, -15180, -15172, -15180, -15206, -15248, -15306, -15377,
    -15461, -15555, -15656, -15762, -15870, -15977, -16079, -16172, -16253, -16317, -16362, -16383,
    -16377, -16340, -16269, -16159, -16009, -15816, -15577, -15289, -14953, -14565, -14126, -13634,
    -13091, -12496, -11849, -11154, -10411, -9623, -8791, -7921, -7014, -6075, -5107, -4115, -3104,
    -2077, -1041, 0, 1041, 2077, 3104, 4115, 5107, 6075, 7014, 7921, 8791, 9623, 10411, 11154,
    11849, 12496, 13091, 13634, 14126, 14565, 14953, 15289, 15577, 15816, 16009, 16159, 16269,
    16340, 16377, 16383, 16362, 16317, 16253, 16172, 16079, 15977, 15870, 15762, 15656, 15555,
    15461, 15377, 15306, 15248, 15206, 15180, 15172, 15180, 15206, 15248, 15306, 15377, 15461,
    15555, 15656, 15762, 15870, 15977, 16079, 16172, 16253, 16317, 16362, 16383, 16377, 16340,
    16269, 16159, 16009, 15816, 15577, 15289, 14953, 14565, 14126, 13634, 13091, 12496, 11849,
    11154, 10411, 9623, 8791, 7921, 7014, 6075, 5107, 4115, 3104, 2077, 1041, 0, -1041, -2077,
    -3104, -4115, -5107, -6075, -7014, -7921, -8791, -9623, -10411, -11154, -11849, -12496, -13091,
];

const R_SIN3PHA_B_M1_TABLE: [i16; 181] = [
    15172, 15180, 15206, 15248, 15306, 15377, 15461, 15555, 15656, 15762, 15870, 15977, 16079,
    16172, 16253, 16317, 16362, 16383, 16377, 16340, 16269, 16159, 16009, 15816, 15577, 15289,
    14953, 14565, 14126, 13634, 13091, 12496, 11849, 11154, 10411, 9623, 8791, 7921, 7014, 6075,
    5107, 4115, 3104, 2077, 1041, 0, -1041, -2077, -3104, -4115, -5107, -6075, -7014, -7921, -8791,
    -9623, -10411, -11154, -11849, -12496, -13091, -13634, -14126, -14565, -14953, -15289, -15577,
    -15816, -16009, -16159, -16269, -16340, -16377, -16383, -16362, -16317, -16253, -16172, -16079,
    -15977, -15870, -15762, -15656, -15555, -15461, -15377, -15306, -15248, -15206, -15180, -15172,
    -15180, -15206, -15248, -15306, -15377, -15461, -15555, -15656, -15762, -15870, -15977, -16079,
    -16172, -16253, -16317, -16362, -16383, -16377, -16340, -16269, -16159, -16009, -15816, -15577,
    -15289, -14953, -14565, -14126, -13634, -13091, -12496, -11849, -11154, -10411, -9623, -8791,
    -7921, -7014, -6075, -5107, -4115, -3104, -2077, -1041, 0, 1041, 2077, 3104, 4115, 5107, 6075,
    7014, 7921, 8791, 9623, 10411, 11154, 11849, 12496, 13091, 13634, 14126, 14565, 14953, 15289,
    15577, 15816, 16009, 16159, 16269, 16340, 16377, 16383, 16362, 16317, 16253, 16172, 16079,
    15977, 15870, 15762, 15656, 15555, 15461, 15377, 15306, 15248, 15206, 15180, 15172,
];

const R_SIN3PHA_C_M1_TABLE: [i16; 181] = [
    -13091, -12496, -11849, -11154, -10411, -9623, -8791, -7921, -7014, -6075, -5107, -4115, -3104,
    -2077, -1041, 0, 1041, 2077, 3104, 4115, 5107, 6075, 7014, 7921, 8791, 9623, 10411, 11154,
    11849, 12496, 13091, 13634, 14126, 14565, 14953, 15289, 15577, 15816, 16009, 16159, 16269,
    16340, 16377, 16383, 16362, 16317, 16253, 16172, 16079, 15977, 15870, 15762, 15656, 15555,
    15461, 15377, 15306, 15248, 15206, 15180, 15172, 15180, 15206, 15248, 15306, 15377, 15461,
    15555, 15656, 15762, 15870, 15977, 16079, 16172, 16253, 16317, 16362, 16383, 16377, 16340,
    16269, 16159, 16009, 15816, 15577, 15289, 14953, 14565, 14126, 13634, 13091, 12496, 11849,
    11154, 10411, 9623, 8791, 7921, 7014, 6075, 5107, 4115, 3104, 2077, 1041, 0, -1041, -2077,
    -3104, -4115, -5107, -6075, -7014, -7921, -8791, -9623, -10411, -11154, -11849, -12496, -13091,
    -13634, -14126, -14565, -14953, -15289, -15577, -15816, -16009, -16159, -16269, -16340, -16377,
    -16383, -16362, -16317, -16253, -16172, -16079, -15977, -15870, -15762, -15656, -15555, -15461,
    -15377, -15306, -15248, -15206, -15180, -15172, -15180, -15206, -15248, -15306, -15377, -15461,
    -15555, -15656, -15762, -15870, -15977, -16079, -16172, -16253, -16317, -16362, -16383, -16377,
    -16340, -16269, -16159, -16009, -15816, -15577, -15289, -14953, -14565, -14126, -13634, -13091,
];

const IQ_MAXSCA_M1_TABLE: [u16; 50] = [
    65535, 65523, 65484, 65418, 65326, 65207, 65062, 64890, 64691, 64465, 64211, 63930, 63620,
    63281, 62913, 62516, 62088, 61630, 61140, 60618, 60062, 59473, 58848, 58187, 57489, 56752,
    55974, 55155, 54291, 53381, 52422, 51413, 50349, 49227, 48043, 46792, 45470, 44069, 42581,
    40997, 39307, 37494, 35541, 33422, 31105, 28540, 25655, 22323, 18304, 12974,
];

const Z_COMMUTMAP_M1_TABLE: [i8; 18] = [-1, 1, 0, -1, 0, 1, 0, -1, 1, 1, -1, 0, 1, 0, -1, 0, 1, -1];

const VEC_HALLTOPOS_VALUE: [i8; 8] = [0, 2, 0, 1, 4, 3, 5, 0];

const VQ_MAX_M1_TABLE: [i16; 46] = [
    14400, 14396, 14386, 14368, 14343, 14311, 14271, 14225, 14171, 14109, 14040, 13963, 13879,
    13786, 13685, 13576, 13459, 13333, 13198, 13053, 12900, 12736, 12562, 12377, 12181, 11973,
    11753, 11520, 11273, 11011, 10733, 10438, 10124, 9790, 9433, 9051, 8640, 8196, 7713, 7184,
    6597, 5935, 5170, 4245, 3019, 0,
];

const VQ_MAX_XA_TABLE: [i16; 46] = [
    0, 320, 640, 960, 1280, 1600, 1920, 2240, 2560, 2880, 3200, 3520, 3840, 4160, 4480, 4800, 5120,
    5440, 5760, 6080, 6400, 6720, 7040, 7360, 7680, 8000, 8320, 8640, 8960, 9280, 9600, 9920,
    10240, 10560, 10880, 11200, 11520, 11840, 12160, 12480, 12800, 13120, 13440, 13760, 14080,
    14400,
];

/// Pinned parameter set: `rtP_Left` plus `util.c` overrides and the jbot patch.
#[derive(Clone, Copy, Debug)]
pub struct GeneratedParams {
    pub dV_open_rate: i32,
    pub dz_cnt_trns_det_hi: i16,
    pub dz_cnt_trns_det_lo: i16,
    pub n_cruise_mot_tgt: i16,
    pub z_max_cnt_rst: i16,
    pub cf_speed_coef: u16,
    pub t_err_dequal: u16,
    pub t_err_qual: u16,
    pub Vd_max: i16,
    pub Vq_max_M1: [i16; 46],
    pub Vq_max_XA: [i16; 46],
    pub a_pha_adv_max: i16,
    pub i_max: i16,
    pub id_field_weak_max: i16,
    pub n_comm_acv_lo: i16,
    pub n_comm_deacv_hi: i16,
    pub n_field_weak_auth_hi: i16,
    pub n_field_weak_auth_lo: i16,
    pub n_max: i16,
    pub n_std_still_det: i16,
    pub r_err_inp_tgt_thres: i16,
    pub r_field_weak_hi: i16,
    pub r_field_weak_lo: i16,
    pub cf_Kb_lim_prot: u16,
    pub cf_idKp: u16,
    pub cf_iqKp: u16,
    pub cf_nKp: u16,
    pub cf_curr_filt: u16,
    pub cf_idKi: u16,
    pub cf_iqKi: u16,
    pub cf_iqKi_lim_prot: u16,
    pub cf_nKi: u16,
    pub cf_nKi_lim_prot: u16,
    pub n_pole_pairs: u8,
    pub z_ctrl_typ_sel: u8,
    pub z_sel_pha_cur_meas_ABC: u8,
    pub b_angle_meas_ena: bool,
    pub b_cruise_ctrl_ena: bool,
    pub b_diag_ena: bool,
    pub b_field_weak_ena: bool,
}

impl GeneratedParams {
    pub const fn pinned() -> Self {
        Self {
            dV_open_rate: 12288,
            dz_cnt_trns_det_hi: 40,
            dz_cnt_trns_det_lo: 20,
            n_cruise_mot_tgt: 0,
            z_max_cnt_rst: 2000,
            cf_speed_coef: 10667,
            t_err_dequal: 9600,
            t_err_qual: 1280,
            Vd_max: 14400,
            Vq_max_M1: VQ_MAX_M1_TABLE,
            Vq_max_XA: VQ_MAX_XA_TABLE,
            a_pha_adv_max: 400,
            i_max: 8000,
            id_field_weak_max: 4000,
            n_comm_acv_lo: 240,
            n_comm_deacv_hi: 480,
            n_field_weak_auth_hi: 6400,
            n_field_weak_auth_lo: 4800,
            n_max: 6400,
            n_std_still_det: 48,
            r_err_inp_tgt_thres: 9600,
            r_field_weak_hi: 16000,
            r_field_weak_lo: 12000,
            cf_Kb_lim_prot: 768,
            cf_idKp: 819,
            cf_iqKp: 1229,
            cf_nKp: 4833,
            cf_curr_filt: 7864,
            cf_idKi: 737,
            cf_iqKi: 1229,
            cf_iqKi_lim_prot: 737,
            cf_nKi: 251,
            cf_nKi_lim_prot: 246,
            n_pole_pairs: 15,
            z_ctrl_typ_sel: 2,
            z_sel_pha_cur_meas_ABC: 0,
            b_angle_meas_ena: false,
            b_cruise_ctrl_ena: false,
            b_diag_ena: true,
            b_field_weak_ena: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Inputs {
    pub b_motEna: bool,
    pub z_ctrlModReq: u8,
    pub r_inpTgt: i16,
    pub b_hallA: bool,
    pub b_hallB: bool,
    pub b_hallC: bool,
    pub i_phaAB: i16,
    pub i_phaBC: i16,
    pub i_DCLink: i16,
    pub a_mechAngle: i16,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Outputs {
    pub dc_pha_a: i16,
    pub dc_pha_b: i16,
    pub dc_pha_c: i16,
    pub z_err_code: u8,
    pub n_mot: i16,
    pub a_elec_angle: i16,
    pub iq: i16,
    pub id: i16,
}

#[derive(Clone, Copy, Debug, Default)]
struct PiState {
    resettable: i32,
    ic_load: bool,
    unit_delay1: bool,
}

/// Observability snapshot of one controller after a completed step.
///
/// Values are the internal fixed-point quantities before output scaling so a
/// calibration host can correlate controller decisions with the phase duty.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MotorTrace {
    pub mode: u8,
    pub scheduler: u8,
    pub angle_q6: i16,
    pub speed_rpm: i16,
    pub target: i16,
    pub iq: i16,
    pub id: i16,
    pub vq: i16,
    pub vd: i16,
    pub integral_q: i32,
    pub integral_d: i32,
    pub phase_a: i16,
    pub phase_b: i16,
    pub phase_c: i16,
    pub err_code: u8,
}

#[derive(Clone, Copy, Default)]
struct BackCalcState {
    delta: i32,
    integral: i32,
}

#[derive(Clone, Copy, Default)]
struct DebounceState {
    edge_prev: bool,
    qual_counter: u16,
    dequal_counter: u16,
    out: bool,
}

#[inline]
fn sat16(v: i32) -> i16 {
    v.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

#[inline]
fn abs16(v: i16) -> i16 {
    if v < 0 {
        (v as i32).wrapping_neg() as i16
    } else {
        v
    }
}

#[inline]
fn sign_i16(v: i16) -> i16 {
    if v < 0 {
        -1
    } else {
        (v > 0) as i16
    }
}

#[inline]
fn sign_i32(v: i32) -> i32 {
    if v < 0 {
        -1
    } else {
        (v > 0) as i32
    }
}

#[inline]
fn extend28(v: i32) -> i32 {
    if v & 0x0800_0000 != 0 {
        v | -0x0800_0000
    } else {
        v & 0x07ff_ffff
    }
}

fn plook_i16(u: i16, bp0: i16, bp_space: u16, max_index: u32) -> u8 {
    if u <= bp0 {
        0
    } else {
        let d = (u as i32 - bp0 as i32) as u16;
        let index = (d as u32 / bp_space as u32) as u16;
        if (index as u32) < max_index {
            index as u8
        } else {
            max_index as u8
        }
    }
}

fn plook_u16(u: u16, bp0: u16, bp_space: u16, max_index: u32) -> u8 {
    if u <= bp0 {
        0
    } else {
        let index = ((u as u32 - bp0 as u32) / bp_space as u32) as u16;
        if (index as u32) < max_index {
            index as u8
        } else {
            max_index as u8
        }
    }
}

fn div_floor(numerator: i32, denominator: i32) -> i32 {
    let rem = numerator.wrapping_rem(denominator);
    let adjust = ((numerator < 0) != (denominator < 0)) && rem != 0;
    numerator.wrapping_div(denominator) - adjust as i32
}

fn counter_inc(inc: i16, max: i16, reset: bool, state: &mut i16) -> i16 {
    let prev = if reset { 0 } else { *state };
    let out = inc.wrapping_add(prev);
    *state = if out < max { out } else { max };
    out
}

fn counter_u16(inc: u16, max: u16, reset: bool, state: &mut u16) {
    let prev = if reset { 0 } else { *state };
    let out = inc.wrapping_add(prev);
    *state = if out < max { out } else { max };
}

fn either_edge(input: bool, state: &mut bool) -> bool {
    let out = input != *state;
    *state = input;
    out
}

fn debounce_filter(input: bool, t_acv: u16, t_deacv: u16, state: &mut DebounceState) {
    let edge = either_edge(input, &mut state.edge_prev);
    if input && !state.out {
        counter_u16(1, t_acv, edge, &mut state.qual_counter);
        state.out = state.qual_counter > t_acv || state.out;
    } else if !input && state.out {
        counter_u16(1, t_deacv, edge, &mut state.dequal_counter);
        state.out = !(state.dequal_counter > t_deacv) && state.out;
    }
}

fn low_pass_filter(input: [i16; 2], coef: u16, state: &mut [i32; 2], out: &mut [i16; 2]) {
    for i in 0..2 {
        let mut sum = input[i] as i32 - (state[i] >> 16);
        sum = sat16(sum) as i32;
        sum = (coef as i32).wrapping_mul(sum).wrapping_add(state[i]);
        out[i] = (sum >> 16) as i16;
        state[i] = sum;
    }
}

fn back_calc(
    err: i16,
    gain_i: u16,
    gain_kb: u16,
    sat_max: i16,
    sat_min: i16,
    state: &mut BackCalcState,
) -> i16 {
    let mut sum = ((err as i32) * gain_i as i32) >> 4;
    if sum < 0 && state.delta < i32::MIN.wrapping_sub(sum) {
        sum = i32::MIN;
    } else if sum > 0 && state.delta > i32::MAX.wrapping_sub(sum) {
        sum = i32::MAX;
    } else {
        sum = sum.wrapping_add(state.delta);
    }
    sum = sum.wrapping_add(state.integral);
    let converted = (sum >> 12) as i16;
    let out = converted.clamp(sat_min, sat_max);
    state.delta = (out as i32 - converted as i32).wrapping_mul(gain_kb as i32) as i16 as i32;
    state.integral = sum;
    out
}

// The id PI keeps its integral in a 32-bit state; rtu_init is already 32-bit.
fn pi_clamp_fixdt(
    err: i16,
    p_gain: u16,
    i_gain: u16,
    init: i32,
    sat_max: i16,
    sat_min: i16,
    ext_lim: i32,
    out: &mut i16,
    state: &mut PiState,
) {
    let mut q0 = (err as i32) * i_gain as i32;
    if q0 < 0 && ext_lim < i32::MIN.wrapping_sub(q0) {
        q0 = i32::MIN;
    } else if q0 > 0 && ext_lim > i32::MAX.wrapping_sub(q0) {
        q0 = i32::MAX;
    } else {
        q0 = q0.wrapping_add(ext_lim);
    }

    if state.ic_load {
        state.resettable = init;
    }
    let tmp_integral = if state.unit_delay1 { 0 } else { q0 };
    let sum = tmp_integral.wrapping_add(state.resettable);

    let mut tmp = ((err as i32) * p_gain as i32) >> 11;
    tmp = sat16(tmp) as i32;
    tmp = (((sum >> 16) << 1).wrapping_add(tmp)) >> 1;
    tmp = sat16(tmp) as i32;

    let lower = (tmp as i16) > sat_max;
    let upper = (tmp as i16) < sat_min;
    *out = if lower {
        sat_max
    } else if upper {
        sat_min
    } else {
        tmp as i16
    };

    let q_sign = sign_i32(q0);
    let u_sign = sign_i16(tmp as i16) as i32;
    state.unit_delay1 = q_sign == u_sign && (lower || upper);
    state.ic_load = false;
    state.resettable = sum;
}

// The speed PI scales its 16-bit initial condition to Q16 before storing it.
fn pi_clamp_fixdt_l(
    err: i16,
    p_gain: u16,
    i_gain: u16,
    init: i16,
    sat_max: i16,
    sat_min: i16,
    ext_lim: i32,
    out: &mut i16,
    state: &mut PiState,
) {
    let mut q0 = (err as i32) * i_gain as i32;
    if q0 < 0 && ext_lim < i32::MIN.wrapping_sub(q0) {
        q0 = i32::MIN;
    } else if q0 > 0 && ext_lim > i32::MAX.wrapping_sub(q0) {
        q0 = i32::MAX;
    } else {
        q0 = q0.wrapping_add(ext_lim);
    }

    if state.ic_load {
        state.resettable = (init as i32) << 16;
    }
    let tmp_integral = if state.unit_delay1 { 0 } else { q0 };
    let sum = tmp_integral.wrapping_add(state.resettable);

    let mut tmp = ((err as i32) * p_gain as i32) >> 11;
    tmp = sat16(tmp) as i32;
    tmp = (((sum >> 16) << 1).wrapping_add(tmp)) >> 1;
    tmp = sat16(tmp) as i32;

    let lower = (tmp as i16) > sat_max;
    let upper = (tmp as i16) < sat_min;
    *out = if lower {
        sat_max
    } else if upper {
        sat_min
    } else {
        tmp as i16
    };

    let q_sign = sign_i32(q0);
    let u_sign = sign_i16(tmp as i16) as i32;
    state.unit_delay1 = q_sign == u_sign && (lower || upper);
    state.ic_load = false;
    state.resettable = sum;
}

// The iq PI rounds the integral division up for negatives and keeps a 16-bit
// accumulator; it must not be unified with the other two PI variants.
fn pi_clamp_fixdt_k(
    err: i16,
    p_gain: u16,
    i_gain: u16,
    init: i16,
    sat_max: i16,
    sat_min: i16,
    ext_lim: i32,
    out: &mut i16,
    state: &mut PiState,
) {
    let mut q0 = (err as i32) * i_gain as i32;
    if q0 < 0 && ext_lim < i32::MIN.wrapping_sub(q0) {
        q0 = i32::MIN;
    } else if q0 > 0 && ext_lim > i32::MAX.wrapping_sub(q0) {
        q0 = i32::MAX;
    } else {
        q0 = q0.wrapping_add(ext_lim);
    }

    if state.ic_load {
        state.resettable = init as i32;
    }
    let integral: i16 = if state.unit_delay1 {
        0
    } else {
        let rounded = if q0 < 0 { q0.wrapping_add(65535) } else { q0 };
        (rounded >> 16) as i16
    };
    let sum: i16 = integral.wrapping_add(state.resettable as i16);

    let mut tmp = ((err as i32) * p_gain as i32) >> 11;
    tmp = sat16(tmp) as i32;
    tmp = ((sum as i32 * 2).wrapping_add(tmp)) >> 1;
    tmp = sat16(tmp) as i32;

    let lower = (tmp as i16) > sat_max;
    let upper = (tmp as i16) < sat_min;
    *out = if lower {
        sat_max
    } else if upper {
        sat_min
    } else {
        tmp as i16
    };

    let q_sign = sign_i32(q0);
    let u_sign = sign_i16(tmp as i16);
    state.unit_delay1 = q_sign == u_sign as i32 && (lower || upper);
    state.ic_load = false;
    state.resettable = sum as i32;
}

/// State of one generated controller instance; mirrors the C `DW` structure.
#[derive(Clone, Default)]
pub struct GeneratedMotor {
    pi_i: PiState,
    pi_l4: PiState,
    pi_kh: PiState,
    back_i: BackCalcState,
    back_1: BackCalcState,
    back_j: BackCalcState,
    debounce: DebounceState,
    filter: [i32; 2],
    counter: i16,
    divide1: i32,
    rate_limit_state: i32,
    gain4_e: [i16; 3],
    dtc: [i16; 2],
    z_counter_raw_prev: i16,
    merge: i16,
    switch1: i16,
    vd_max1: i16,
    gain3: i16,
    vq_max_m1: i16,
    gain5: i16,
    i_max: i16,
    divide1_n: i16,
    gain1: i16,
    gain4: i16,
    switch2_i: i16,
    switch2_o: i16,
    switch2_a: i16,
    divide3: i16,
    merge1: i16,
    abs1: i16,
    abs5_h: i16,
    divide11: i16,
    r_sin_M1: i16,
    r_cos_M1: i16,
    ud3_dstate: i16,
    ud4_dstate: i16,
    ud2_dstate: i16,
    ud3_dstate_o: i16,
    ud5_dstate: i16,
    ud4_dstate_e: i16,
    ud4_dstate_eu: i16,
    switch2_e: i8,
    ud2_dstate_b_dir: i8,
    if1_active: i8,
    if2_active: i8,
    if1_active_j: i8,
    switch_case_active: i8,
    if1_active_a: i8,
    if1_active_o: i8,
    switch_case_active_d: i8,
    if2_active_f: i8,
    if2_active_a: i8,
    z_ctrl_mod: u8,
    hall_prev_a: u8,
    hall_prev_b: u8,
    hall_prev_c: u8,
    err_latched: u8,
    err_out: u8,
    err_edge_prev: bool,
    is_active_c1: u8,
    is_c1: u8,
    is_active: u8,
    merge_p: bool,
    dz_cnt_trns_det: bool,
    ud2_dstate_c: bool,
    ud5_dstate_m: bool,
    ud6_dstate: bool,
    rising_edge: bool,
    ud1_dstate_n: bool,
    n_comm_deacv_mode: bool,
    dz_cnt_trns_det_mode: bool,
    // Observability only: locals needed by `trace()` cannot be read back from
    // the generated state after the step has rotated the scheduler tokens.
    trace_angle: i16,
    trace_speed: i16,
    trace_target: i16,
    trace_scheduler: u8,
}

impl GeneratedMotor {
    pub fn new() -> Self {
        let mut motor = Self::default();
        motor.initialize();
        motor
    }

    pub fn initialize(&mut self) {
        self.if1_active = -1;
        self.if2_active_a = -1;
        self.if2_active_f = -1;
        self.if1_active_o = -1;
        self.switch_case_active_d = -1;
        self.if1_active_j = -1;
        self.switch_case_active = -1;
        self.if1_active_a = -1;
        self.if2_active = -1;
        self.ud3_dstate = 2000;
        self.ud2_dstate_c = true;
        self.z_counter_raw_prev = 2000;
        self.counter = 2000;
        self.debounce = DebounceState::default();
        self.rising_edge = true;
        self.back_i = BackCalcState {
            delta: 0,
            integral: 65536000,
        };
        self.back_1 = BackCalcState {
            delta: 0,
            integral: 65536000,
        };
        self.back_j = BackCalcState {
            delta: 0,
            integral: 58982400,
        };
        self.vd_max1 = 14400;
        self.gain3 = -14400;
        self.vq_max_m1 = 14400;
        self.gain5 = -14400;
        self.i_max = 12000;
        self.gain4 = -12000;
        self.divide1_n = 12000;
        self.gain1 = -12000;
        self.pi_l4.ic_load = true;
        self.pi_kh.ic_load = true;
        self.pi_i.ic_load = true;
    }

    /// Returns the controller internals captured at the end of the last step.
    pub fn trace(&self) -> MotorTrace {
        MotorTrace {
            mode: self.z_ctrl_mod,
            scheduler: self.trace_scheduler,
            angle_q6: self.trace_angle >> 6,
            speed_rpm: self.trace_speed >> 4,
            target: self.trace_target,
            iq: self.dtc[0],
            id: self.dtc[1],
            vq: self.merge,
            vd: self.switch1,
            integral_q: self.pi_kh.resettable,
            integral_d: self.pi_i.resettable,
            phase_a: self.gain4_e[0],
            phase_b: self.gain4_e[1],
            phase_c: self.gain4_e[2],
            err_code: self.err_out,
        }
    }

    pub fn step(&mut self, p: &GeneratedParams, u: Inputs) -> Outputs {
        // Hall code and edge-triggered direction/speed estimation.
        let sum_hall = (u.b_hallA as u8) << 2 | (u.b_hallB as u8) << 1 | u.b_hallC as u8;
        let hall_edge =
            (u.b_hallA ^ u.b_hallB ^ u.b_hallC ^ (self.hall_prev_a != 0) ^ (self.hall_prev_b != 0))
                ^ (self.hall_prev_c != 0);

        if hall_edge {
            let prev_dir = self.switch2_e;
            let hall_delta = (VEC_HALLTOPOS_VALUE[sum_hall as usize] as i32
                - self.ud2_dstate_b_dir as i32) as i8;
            self.switch2_e = if hall_delta == 1 || hall_delta == -5 {
                1
            } else {
                -1
            };
            self.ud2_dstate_b_dir = VEC_HALLTOPOS_VALUE[sum_hall as usize];

            self.z_counter_raw_prev = self.ud3_dstate;
            let raw_delta = self.z_counter_raw_prev.wrapping_sub(self.ud4_dstate);
            let mut abs_delta = abs16(raw_delta);
            if abs_delta >= p.dz_cnt_trns_det_hi {
                self.dz_cnt_trns_det_mode = true;
            } else if abs_delta <= p.dz_cnt_trns_det_lo {
                self.dz_cnt_trns_det_mode = false;
            }
            self.dz_cnt_trns_det = self.dz_cnt_trns_det_mode;

            let direction_changed = self.switch2_e != prev_dir;
            abs_delta = if direction_changed && self.ud1_dstate_n {
                0
            } else if direction_changed {
                self.ud4_dstate
            } else if self.dz_cnt_trns_det {
                (((p.cf_speed_coef as i32) << 4) / self.z_counter_raw_prev as i32) as i16
            } else {
                let numerator = (p.cf_speed_coef as i32) << 2;
                let denominator = self
                    .ud2_dstate
                    .wrapping_add(self.ud3_dstate_o)
                    .wrapping_add(self.ud5_dstate)
                    .wrapping_add(self.z_counter_raw_prev);
                ((numerator << 4) / denominator as i32) as i16
            };
            self.divide11 = (abs_delta as i32 * self.switch2_e as i32) as i16;
            self.ud4_dstate = self.z_counter_raw_prev;
            self.ud2_dstate = self.ud3_dstate_o;
            self.ud3_dstate_o = self.ud5_dstate;
            self.ud5_dstate = self.z_counter_raw_prev;
            self.ud1_dstate_n = direction_changed;
        }

        let counter_out = counter_inc(1, p.z_max_cnt_rst, hall_edge, &mut self.counter);
        let switch2_speed = if counter_out > p.z_max_cnt_rst {
            0
        } else {
            self.divide11
        };
        let abs5_speed = abs16(switch2_speed);
        if abs5_speed >= p.n_comm_deacv_hi {
            self.n_comm_deacv_mode = true;
        } else if abs5_speed <= p.n_comm_acv_lo {
            self.n_comm_deacv_mode = false;
        }
        let comm_detected = p.b_angle_meas_ena || (self.n_comm_deacv_mode && !self.dz_cnt_trns_det);

        let sched_ud2 = self.ud2_dstate_c;
        let sched_ud5 = self.ud5_dstate_m;

        let mut data_type_conversion2 = (u.r_inpTgt as i32 * 16) as i16;

        let mut gain3_i = (u.i_phaAB as i32) << 4;
        let mut sat_a = if gain3_i >= 27200 {
            27200
        } else if gain3_i <= -27200 {
            -27200
        } else {
            gain3_i as i16
        };
        gain3_i = (u.i_phaBC as i32) << 4;
        let sat_b = if gain3_i >= 27200 {
            27200
        } else if gain3_i <= -27200 {
            -27200
        } else {
            gain3_i as i16
        };

        // Electrical angle: Hall interpolation or measured mechanical angle.
        let angle = if !p.b_angle_meas_ena {
            let mut merge_m = counter_out.min(self.z_counter_raw_prev);
            let hall_pos = if self.switch2_e == 1 {
                VEC_HALLTOPOS_VALUE[sum_hall as usize]
            } else {
                VEC_HALLTOPOS_VALUE[sum_hall as usize] + 1
            };
            if comm_detected {
                let fraction = ((merge_m as i32) << 14) / self.z_counter_raw_prev as i32;
                merge_m =
                    ((fraction * self.switch2_e as i32 + (hall_pos as i32 * 16384)) >> 2) as i16;
            } else {
                merge_m = (hall_pos as i32 * 4096) as i16;
            }
            if merge_m <= 0 {
                merge_m = 0;
            }
            ((15 * merge_m as i32) >> 4) as i16
        } else {
            let tmp = u.a_mechAngle as i32 * p.n_pole_pairs as i32 - 480;
            let offset = ((div_floor(tmp, 5760) as i16 as i32 * 360) as i16 as i32) << 4;
            let wrapped = tmp - offset;
            (wrapped as i16 as i32 * 4) as i16
        };

        // Forward Clarke/Park runs on every tick when FOC is selected.
        let last_if1 = self.if1_active;
        let if1_active: i8 = if p.z_ctrl_typ_sel == 2 { 0 } else { -1 };
        self.if1_active = if1_active;
        if last_if1 != if1_active && last_if1 == 0 {
            if self.if2_active_a == 0 {
                self.dtc[0] = 0;
                self.abs5_h = 0;
                self.dtc[1] = 0;
            }
            self.if2_active_a = -1;
            self.r_sin_M1 = 0;
            self.r_cos_M1 = 0;
            self.dtc[0] = 0;
            self.dtc[1] = 0;
            self.abs5_h = 0;
        }

        if if1_active == 0 {
            let alpha: i16;
            // The retained phase axis stays in rtb_Saturation across the
            // Clarke stage; only the BC measurement variant replaces it.
            let mut beta = sat_a;
            if p.z_sel_pha_cur_meas_ABC == 0 {
                let a_term = 18919 * sat_a as i32;
                let b_term = 18919 * sat_b as i32;
                let mut combined = ((if a_term < 0 { 32767 } else { 0 }) + a_term) >> 15;
                combined += (((if b_term < 0 { 16383 } else { 0 }) + b_term) >> 14) as i16 as i32;
                alpha = sat16(combined);
            } else if p.z_sel_pha_cur_meas_ABC == 1 {
                let mut diff = sat_a as i32 - sat_b as i32;
                diff = sat16(diff) as i32;
                diff *= 18919;
                alpha = (((if diff < 0 { 32767 } else { 0 }) + diff) >> 15) as i16;
                beta = (-(sat_a as i32) - sat_b as i32).clamp(-32768, 32767) as i16;
            } else {
                let a_term = 18919 * sat_a as i32;
                let b_term = 18919 * sat_b as i32;
                let mut combined = ((if a_term < 0 { 32767 } else { 0 }) + a_term) >> 15;
                combined += (((if b_term < 0 { 16383 } else { 0 }) + b_term) >> 14) as i16 as i32;
                alpha = sat16(-combined);
            }

            let index = plook_i16(angle, 0, 128, 180);
            self.r_sin_M1 = R_SIN_M1_TABLE[index as usize];
            self.r_cos_M1 = R_COS_M1_TABLE[index as usize];

            let last_if2_a = self.if2_active_a;
            let if2_a: i8 = if u.b_motEna { 0 } else { -1 };
            self.if2_active_a = if2_a;
            if last_if2_a != if2_a && last_if2_a == 0 {
                self.dtc[0] = 0;
                self.abs5_h = 0;
                self.dtc[1] = 0;
            }
            if if2_a == 0 {
                if last_if2_a != 0 {
                    self.filter = [0, 0];
                }
                let mut filter_in = [0i16; 2];
                let d = ((alpha as i32 * self.r_cos_M1 as i32) >> 14)
                    - ((beta as i32 * self.r_sin_M1 as i32) >> 14);
                filter_in[0] = sat16(d);
                let q = ((beta as i32 * self.r_cos_M1 as i32) >> 14)
                    + ((alpha as i32 * self.r_sin_M1 as i32) >> 14);
                filter_in[1] = sat16(q);
                low_pass_filter(filter_in, p.cf_curr_filt, &mut self.filter, &mut self.dtc);
                self.abs5_h = abs16(self.dtc[0]);
            }
        }

        // Rotating scheduler: diagnostics, limits, then FOC PIs.
        if sched_ud2 {
            if p.b_diag_ena {
                let err_cond = if self.err_latched & 4 != 0 {
                    true
                } else {
                    u.b_motEna
                        && abs5_speed < p.n_std_still_det
                        && abs16(self.ud4_dstate_eu) > p.r_err_inp_tgt_thres
                };
                let code = (((sum_hall == 7) as u8) << 1)
                    + ((sum_hall == 0) as u8)
                    + ((err_cond as u8) << 2);
                debounce_filter(code != 0, p.t_err_qual, p.t_err_dequal, &mut self.debounce);
                // The error output has its own edge detector, separate from
                // the one inside the debounce filter.
                let err_edge = either_edge(self.merge_p, &mut self.err_edge_prev);
                self.err_out = if err_edge { code } else { self.err_latched };
                self.err_latched = self.err_out;
            }

            let force_open = self.merge_p || !u.b_motEna || u.z_ctrlModReq == 0;
            let speed_request = u.z_ctrlModReq == 2 || p.b_cruise_ctrl_ena;
            let torque_request = u.z_ctrlModReq == 3 && !p.b_cruise_ctrl_ena;

            if self.is_active_c1 == 0 {
                self.is_active_c1 = 1;
                self.is_c1 = IN_OPEN;
                self.z_ctrl_mod = OPEN_MODE;
            } else if self.is_c1 == IN_ACTIVE {
                if force_open {
                    self.is_active = IN_NO_ACTIVE_CHILD;
                    self.is_c1 = IN_OPEN;
                    self.z_ctrl_mod = OPEN_MODE;
                } else {
                    match self.is_active {
                        IN_SPEED_MODE => {
                            self.z_ctrl_mod = SPD_MODE;
                            if !speed_request {
                                self.is_active = IN_NO_ACTIVE_CHILD;
                                if torque_request {
                                    self.is_active = IN_TORQUE_MODE;
                                    self.z_ctrl_mod = TRQ_MODE;
                                } else {
                                    self.is_active = IN_VOLTAGE_MODE;
                                    self.z_ctrl_mod = VLT_MODE;
                                }
                            }
                        }
                        IN_TORQUE_MODE => {
                            self.z_ctrl_mod = TRQ_MODE;
                            if !torque_request {
                                self.is_active = IN_NO_ACTIVE_CHILD;
                                if speed_request {
                                    self.is_active = IN_SPEED_MODE;
                                    self.z_ctrl_mod = SPD_MODE;
                                } else {
                                    self.is_active = IN_VOLTAGE_MODE;
                                    self.z_ctrl_mod = VLT_MODE;
                                }
                            }
                        }
                        _ => {
                            self.z_ctrl_mod = VLT_MODE;
                            if torque_request || speed_request {
                                self.is_active = IN_NO_ACTIVE_CHILD;
                                if torque_request {
                                    self.is_active = IN_TORQUE_MODE;
                                    self.z_ctrl_mod = TRQ_MODE;
                                } else if speed_request {
                                    self.is_active = IN_SPEED_MODE;
                                    self.z_ctrl_mod = SPD_MODE;
                                } else {
                                    self.is_active = IN_VOLTAGE_MODE;
                                    self.z_ctrl_mod = VLT_MODE;
                                }
                            }
                        }
                    }
                }
            } else {
                self.z_ctrl_mod = OPEN_MODE;
                if !force_open && (u.z_ctrlModReq == 1 || speed_request || torque_request) {
                    self.is_c1 = IN_ACTIVE;
                    if torque_request {
                        self.is_active = IN_TORQUE_MODE;
                        self.z_ctrl_mod = TRQ_MODE;
                    } else if speed_request {
                        self.is_active = IN_SPEED_MODE;
                        self.z_ctrl_mod = SPD_MODE;
                    } else {
                        self.is_active = IN_VOLTAGE_MODE;
                        self.z_ctrl_mod = VLT_MODE;
                    }
                }
            }

            let limits = [0i16, p.Vd_max, p.n_max, p.i_max];
            data_type_conversion2 = sat16(data_type_conversion2 as i32);
            if p.z_ctrl_typ_sel == 2 {
                let scale = ((limits[u.z_ctrlModReq as usize] as i32) << 5) / 125;
                let scaled = (scale as u16 as i32) * data_type_conversion2 as i32;
                sat_a = (scaled >> 12) as i16;
            } else if data_type_conversion2 > 16000 {
                sat_a = 16000;
            } else if data_type_conversion2 < -16000 {
                sat_a = -16000;
            } else {
                sat_a = data_type_conversion2;
            }

            let last_if2_f = self.if2_active_f;
            let if2_f: i8 = -((self.z_ctrl_mod != 0) as i8);
            self.if2_active_f = if2_f;
            if if2_f == 0 {
                if if2_f != last_if2_f {
                    self.rising_edge = true;
                    self.rate_limit_state = 0;
                }
                let target = (self.ud4_dstate_eu as i32) << 12;
                let target28 = extend28(target);
                let rising = self.rising_edge;
                self.rising_edge = false;
                let mut limited = if rising {
                    target28
                } else {
                    self.rate_limit_state
                };
                let neg = extend28(-limited);
                let delta = if neg > p.dV_open_rate {
                    p.dV_open_rate
                } else {
                    let lower = extend28(-p.dV_open_rate);
                    if neg < lower {
                        lower
                    } else {
                        neg
                    }
                };
                limited = extend28(limited.wrapping_add(delta));
                self.rate_limit_state = if rising { target28 } else { limited };
                self.merge1 = (limited >> 12) as i16;
            } else {
                self.merge1 = sat_a;
            }
            self.abs1 = abs16(self.merge1);
        } else if sched_ud5 {
            if p.b_field_weak_ena {
                let mut target = data_type_conversion2;
                if target < 0 {
                    target = abs16(target);
                }
                target = target.clamp(p.r_field_weak_lo, p.r_field_weak_hi);
                let lo = ((target as i32 - p.r_field_weak_lo as i32) << 15)
                    / (p.r_field_weak_hi as i32 - p.r_field_weak_lo as i32);
                let speed = self
                    .abs1
                    .clamp(p.n_field_weak_auth_lo, p.n_field_weak_auth_hi);
                let hi = ((speed as i32 - p.n_field_weak_auth_lo as i32) << 15)
                    / (p.n_field_weak_auth_hi as i32 - p.n_field_weak_auth_lo as i32);
                let weight = lo.min(hi) as u16;
                let max_id = if p.z_ctrl_typ_sel == 2 {
                    p.id_field_weak_max
                } else {
                    p.a_pha_adv_max
                };
                self.divide3 = ((max_id as i32 * weight as i32) >> 15) as i16;
            }

            let last_if1_o = self.if1_active_o;
            let if1_o: i8 = if p.z_ctrl_typ_sel == 2 { 0 } else { -1 };
            self.if1_active_o = if1_o;
            if last_if1_o != if1_o && last_if1_o == 0 {
                self.switch_case_active_d = -1;
            }
            if if1_o == 0 {
                self.vd_max1 = p.Vd_max;
                self.gain3 = -self.vd_max1;
                self.vq_max_m1 = p.Vq_max_M1[plook_i16(
                    abs16(self.switch1),
                    p.Vq_max_XA[0],
                    (p.Vq_max_XA[1] - p.Vq_max_XA[0]) as u16,
                    45,
                ) as usize];
                self.gain5 = -self.vq_max_m1;
                self.i_max = p.i_max;
                let mut scaled = (self.divide3 as i32) << 16;
                scaled = if scaled == i32::MIN && self.i_max == -1 {
                    i32::MAX
                } else {
                    scaled / self.i_max as i32
                };
                let table_index = scaled.clamp(0, 65535) as u16;
                let idx = plook_u16(table_index, 0, 1311, 49);
                self.divide1_n =
                    ((IQ_MAXSCA_M1_TABLE[idx as usize] as i32 * self.i_max as i32) >> 16) as i16;
                self.gain1 = -self.divide1_n;

                let last_case = self.switch_case_active_d;
                let case: i8 = match self.z_ctrl_mod {
                    1 => 0,
                    2 => 1,
                    3 => 2,
                    _ => -1,
                };
                self.switch_case_active_d = case;
                if case != last_case {
                    if case == 0 {
                        self.back_i.delta = 0;
                        self.back_i.integral = 65536000;
                        self.back_1.delta = 0;
                        self.back_1.integral = 65536000;
                    } else if case == 2 {
                        self.back_j.delta = 0;
                        self.back_j.integral = 58982400;
                    }
                }
                match case {
                    0 => {
                        self.switch2_a = back_calc(
                            self.divide1_n - self.abs5_h,
                            p.cf_iqKi_lim_prot,
                            p.cf_Kb_lim_prot,
                            self.abs1,
                            0,
                            &mut self.back_i,
                        );
                        self.switch2_o = back_calc(
                            p.n_max - abs5_speed,
                            p.cf_nKi_lim_prot,
                            p.cf_Kb_lim_prot,
                            self.abs1,
                            0,
                            &mut self.back_1,
                        );
                    }
                    1 => {
                        let clamped = self.dtc[0].clamp(self.gain1, self.divide1_n);
                        self.divide1 = (clamped as i32 - self.dtc[0] as i32) as i16 as i32
                            * p.cf_iqKi_lim_prot as i32;
                    }
                    2 => {
                        self.switch2_i = back_calc(
                            p.n_max - abs5_speed,
                            p.cf_nKi_lim_prot,
                            p.cf_Kb_lim_prot,
                            self.vq_max_m1,
                            0,
                            &mut self.back_j,
                        );
                    }
                    _ => {}
                }
                self.gain4 = -self.i_max;
            }
        } else if self.ud6_dstate {
            let last_if1_j = self.if1_active_j;
            let if1_j: i8 = if p.z_ctrl_typ_sel == 2 { 0 } else { -1 };
            self.if1_active_j = if1_j;
            if last_if1_j != if1_j && last_if1_j == 0 {
                self.switch_case_active = -1;
                self.if1_active_a = -1;
            }
            if if1_j == 0 {
                let last_case = self.switch_case_active;
                // VLT mode leaves the generated state at its -1 initial value.
                let case: i8 = match self.z_ctrl_mod {
                    1 => -1,
                    2 => 1,
                    3 => 2,
                    _ => 3,
                };
                self.switch_case_active = case;
                match case {
                    0 => {
                        let mut limit = self.abs1.min(self.switch2_a);
                        limit = limit.max(self.switch2_o);
                        let sign = sign_i16(self.merge1);
                        let vq = (limit as i32 * sign as i32) as i16;
                        self.merge = vq.clamp(self.gain5, self.vq_max_m1);
                    }
                    1 => {
                        if case != last_case {
                            self.pi_l4.resettable = 0;
                            self.pi_l4.ic_load = true;
                            self.pi_l4.unit_delay1 = false;
                        }
                        let cruise = (p.n_cruise_mot_tgt as i32 * 16) as i16;
                        let sat_max;
                        let sat_min;
                        if p.b_cruise_ctrl_ena && cruise != 0 {
                            if cruise > 0 {
                                sat_max = self.vq_max_m1;
                                sat_min = self.merge1.max(self.gain5);
                            } else {
                                sat_max = self.vq_max_m1.min(self.merge1);
                                sat_min = self.gain5;
                            }
                        } else {
                            sat_max = self.vq_max_m1;
                            sat_min = self.gain5;
                        }
                        let target = if p.b_cruise_ctrl_ena {
                            cruise
                        } else {
                            self.merge1
                        };
                        let err = sat16(target as i32 - switch2_speed as i32);
                        pi_clamp_fixdt_l(
                            err,
                            p.cf_nKp,
                            p.cf_nKi,
                            self.ud4_dstate_eu,
                            sat_max,
                            sat_min,
                            self.divide1,
                            &mut self.merge,
                            &mut self.pi_l4,
                        );
                    }
                    2 => {
                        if case != last_case {
                            self.pi_kh.resettable = 0;
                            self.pi_kh.ic_load = true;
                            self.pi_kh.unit_delay1 = false;
                        }
                        let neg_switch2 = -(self.switch2_i as i32) as i16;
                        let clamped = self.merge1.clamp(self.gain1, self.divide1_n);
                        let err = sat16(clamped as i32 - self.dtc[0] as i32);
                        let sat_max = self.vq_max_m1.min(self.switch2_i);
                        let sat_min = neg_switch2.max(self.gain5);
                        pi_clamp_fixdt_k(
                            err,
                            p.cf_iqKp,
                            p.cf_iqKi,
                            self.ud4_dstate_eu,
                            sat_max,
                            sat_min,
                            0,
                            &mut self.merge,
                            &mut self.pi_kh,
                        );
                    }
                    3 => self.merge = self.merge1,
                    _ => {}
                }

                let last_vd = self.if1_active_a;
                let mut vd_active: i8 = -1;
                if comm_detected {
                    vd_active = 0;
                }
                self.if1_active_a = vd_active;
                if vd_active == 0 {
                    if vd_active != last_vd {
                        self.pi_i.resettable = 0;
                        self.pi_i.ic_load = true;
                        self.pi_i.unit_delay1 = false;
                    }
                    let mut target = -(self.divide3 as i32) as i16;
                    target = target.clamp(self.gain4, self.i_max);
                    let err = sat16(target as i32 - self.dtc[1] as i32);
                    pi_clamp_fixdt(
                        err,
                        p.cf_idKp,
                        p.cf_idKi,
                        0,
                        self.vd_max1,
                        self.gain3,
                        0,
                        &mut self.switch1,
                        &mut self.pi_i,
                    );
                }
            }
        }

        // Inverse Clarke/Park runs on every tick for FOC.
        let last_if2 = self.if2_active;
        let if2_active: i8;
        if p.z_ctrl_typ_sel == 2 {
            sat_a = self.merge;
            if2_active = 0;
        } else {
            sat_a = self.merge1;
            if2_active = -1;
        }
        self.if2_active = if2_active;
        if last_if2 != if2_active && last_if2 == 0 {
            self.gain4_e = [0, 0, 0];
        }
        if if2_active == 0 {
            let u_alpha = sat16(
                ((self.switch1 as i32 * self.r_cos_M1 as i32) >> 14)
                    - ((self.merge as i32 * self.r_sin_M1 as i32) >> 14),
            );
            let mut u_beta = sat16(
                ((self.switch1 as i32 * self.r_sin_M1 as i32) >> 14)
                    + ((self.merge as i32 * self.r_cos_M1 as i32) >> 14),
            );
            let mut scaled = u_beta as i32 * 14189;
            u_beta = sat16(
                (((if scaled < 0 { 16383 } else { 0 }) + scaled) >> 14)
                    - ((((if u_alpha < 0 { 1 } else { 0 }) + u_alpha as i32) as i16) >> 1) as i32,
            );
            scaled = u_beta as i32;
            let u_gamma = sat16(-(u_alpha as i32) - scaled);
            let mut min_phase = u_alpha;
            if min_phase >= u_beta {
                min_phase = u_beta;
            }
            if min_phase >= u_gamma {
                min_phase = u_gamma;
            }
            let mut max_phase = u_alpha;
            if max_phase <= u_beta {
                max_phase = u_beta;
            }
            if max_phase <= u_gamma {
                max_phase = u_gamma;
            }
            let common = sat16(min_phase as i32 + max_phase as i32);
            let half_common = common >> 1;
            let a = sat16(u_alpha as i32 - half_common as i32);
            self.gain4_e[0] = ((18919 * a as i32) >> 14) as i16;
            let b = sat16(u_beta as i32 - half_common as i32);
            self.gain4_e[1] = ((18919 * b as i32) >> 14) as i16;
            let c = sat16(u_gamma as i32 - half_common as i32);
            self.gain4_e[2] = ((18919 * c as i32) >> 14) as i16;
        }

        // Output method selection. The C code writes a local phase-C signal
        // here, never the DW Merge1 target used by the PI controllers.
        let phase_c: i16;
        if comm_detected && p.z_ctrl_typ_sel == 2 {
            data_type_conversion2 = self.gain4_e[0];
            sat_a = self.gain4_e[1];
            phase_c = self.gain4_e[2];
        } else if comm_detected && p.z_ctrl_typ_sel == 1 {
            if p.b_field_weak_ena {
                let mut advanced = ((self.divide3 as i32 * self.switch2_e as i32) << 2) as i16;
                advanced = (advanced as i32 + angle as i32) as i16;
                data_type_conversion2 =
                    (advanced as i32 - div_floor(advanced as i32, 23040) * 360 * 64) as i16;
            } else {
                data_type_conversion2 = angle;
            }
            let index = plook_i16(data_type_conversion2, 0, 128, 180);
            let amplitude = sat_a;
            data_type_conversion2 =
                ((amplitude as i32 * R_SIN3PHA_A_M1_TABLE[index as usize] as i32) >> 14) as i16;
            sat_a = ((amplitude as i32 * R_SIN3PHA_B_M1_TABLE[index as usize] as i32) >> 14) as i16;
            phase_c =
                ((amplitude as i32 * R_SIN3PHA_C_M1_TABLE[index as usize] as i32) >> 14) as i16;
        } else {
            let pos = VEC_HALLTOPOS_VALUE[sum_hall as usize].clamp(0, 5);
            let base = pos as usize * 3;
            let commut_amplitude = sat_a;
            data_type_conversion2 =
                (Z_COMMUTMAP_M1_TABLE[base] as i32 * commut_amplitude as i32) as i16;
            sat_a = (Z_COMMUTMAP_M1_TABLE[base + 1] as i32 * commut_amplitude as i32) as i16;
            phase_c = (Z_COMMUTMAP_M1_TABLE[base + 2] as i32 * commut_amplitude as i32) as i16;
        }

        let out = Outputs {
            dc_pha_a: data_type_conversion2 >> 4,
            dc_pha_b: sat_a >> 4,
            dc_pha_c: phase_c >> 4,
            z_err_code: self.err_out,
            n_mot: switch2_speed >> 4,
            a_elec_angle: angle >> 6,
            iq: self.dtc[0],
            id: self.dtc[1],
        };

        // Sample the locals and the scheduler tokens that produced this tick
        // before the rotation below changes the tokens for the next tick.
        self.trace_angle = angle;
        self.trace_speed = switch2_speed;
        self.trace_target = u.r_inpTgt;
        self.trace_scheduler = (self.ud2_dstate_c as u8)
            | ((self.ud5_dstate_m as u8) << 1)
            | ((self.ud6_dstate as u8) << 2);

        self.hall_prev_a = u.b_hallA as u8;
        self.hall_prev_b = u.b_hallB as u8;
        self.hall_prev_c = u.b_hallC as u8;
        self.ud3_dstate = counter_out;
        self.ud4_dstate_e = abs5_speed;
        self.ud2_dstate_c = self.ud6_dstate;
        self.ud5_dstate_m = sched_ud2;
        self.ud6_dstate = sched_ud5;
        self.ud4_dstate_eu = sat_a;

        out
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    const HEADER: &str = "tick,b_motEna,ctrlModReq,inpTgt,hallA,hallB,hallC,phaAB,phaBC,DC_phaA,DC_phaB,DC_phaC,errCode,n_mot,elecAngle,iq,id";

    fn replay(csv: &str, params: &GeneratedParams) -> MotorTrace {
        let mut lines = csv.lines();
        assert_eq!(lines.next().expect("CSV header"), HEADER);

        let mut motor = GeneratedMotor::new();
        let mut rows = 0;
        for line in lines {
            if line.trim().is_empty() {
                continue;
            }
            let f: Vec<&str> = line.split(',').collect();
            let num = |i: usize| -> i64 {
                f[i].trim()
                    .parse()
                    .unwrap_or_else(|_| panic!("bad field {}", f[i]))
            };
            let input = Inputs {
                b_motEna: num(1) != 0,
                z_ctrlModReq: num(2) as u8,
                r_inpTgt: num(3) as i16,
                b_hallA: num(4) != 0,
                b_hallB: num(5) != 0,
                b_hallC: num(6) != 0,
                i_phaAB: num(7) as i16,
                i_phaBC: num(8) as i16,
                i_DCLink: 0,
                a_mechAngle: 0,
            };
            let out = motor.step(params, input);
            let tick = num(0);
            let actual = [
                out.dc_pha_a as i64,
                out.dc_pha_b as i64,
                out.dc_pha_c as i64,
                out.z_err_code as i64,
                out.n_mot as i64,
                out.a_elec_angle as i64,
                out.iq as i64,
                out.id as i64,
            ];
            let expected = [
                num(9),
                num(10),
                num(11),
                num(12),
                num(13),
                num(14),
                num(15),
                num(16),
            ];
            for (field, (got, want)) in [
                "DC_phaA",
                "DC_phaB",
                "DC_phaC",
                "errCode",
                "n_mot",
                "elecAngle",
                "iq",
                "id",
            ]
            .iter()
            .zip(actual.iter().zip(expected.iter()))
            {
                assert_eq!(
                    got, want,
                    "tick {tick}: {field}: got {got}, want {want}; full row: {line}"
                );
            }
            rows += 1;
        }
        assert_eq!(rows, 120, "golden vector must contain 120 rows");
        motor.trace()
    }

    #[test]
    fn golden_left_is_bit_exact() {
        replay(
            include_str!("../../tools/foc_golden/foc_golden_left.csv"),
            &GeneratedParams::pinned(),
        );
    }

    #[test]
    fn golden_right_is_bit_exact() {
        let mut params = GeneratedParams::pinned();
        params.z_sel_pha_cur_meas_ABC = 1;
        replay(
            include_str!("../../tools/foc_golden/foc_golden_right.csv"),
            &params,
        );
    }

    #[test]
    fn trace_is_stable_after_golden_replay() {
        let left = replay(
            include_str!("../../tools/foc_golden/foc_golden_left.csv"),
            &GeneratedParams::pinned(),
        );
        assert_eq!(
            left,
            MotorTrace {
                mode: 3,
                scheduler: 4,
                angle_q6: 0,
                speed_rpm: -1334,
                target: 0,
                iq: -3042,
                id: -6580,
                vq: 1496,
                vd: 1473,
                integral_q: 584,
                integral_d: 10_378_434,
                phase_a: 913,
                phase_b: 2031,
                phase_c: -2031,
                err_code: 0,
            }
        );

        let mut params = GeneratedParams::pinned();
        params.z_sel_pha_cur_meas_ABC = 1;
        let right = replay(
            include_str!("../../tools/foc_golden/foc_golden_right.csv"),
            &params,
        );
        assert_eq!(
            right,
            MotorTrace {
                mode: 3,
                scheduler: 4,
                angle_q6: 0,
                speed_rpm: -1334,
                target: 0,
                iq: -4131,
                id: 4748,
                vq: 1886,
                vd: -874,
                integral_q: 647,
                integral_d: 5_030_025,
                phase_a: -2070,
                phase_b: 2070,
                phase_c: -320,
                err_code: 0,
            }
        );
    }
}
