/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#if !defined(__has_include)
  #define __has_include(...) 1
#endif

#include <stdint.h>

#define ABCE 4
#define XYZE 4
#define XYZE_N  4
#define ABC  3
#define XYZ  3
#define XY   2

#define _AXIS(A) (A##_AXIS)

#define _XMIN_   100
#define _YMIN_   200
#define _ZMIN_   300
#define _XMAX_   101
#define _YMAX_   201
#define _ZMAX_   301
#define _XDIAG_  102
#define _YDIAG_  202
#define _ZDIAG_  302
#define _E0DIAG_ 400
#define _E1DIAG_ 401
#define _E2DIAG_ 402
#define _E3DIAG_ 403
#define _E4DIAG_ 404
#define _E5DIAG_ 405
#define _E6DIAG_ 406
#define _E7DIAG_ 407

#define _FORCE_INLINE_ __attribute__((__always_inline__)) __inline__
#define  FORCE_INLINE  __attribute__((always_inline)) inline
#define _UNUSED      __attribute__((unused))
#define _O0          __attribute__((optimize("O0")))
#define _Os          __attribute__((optimize("Os")))
#define _O1          __attribute__((optimize("O1")))
#define _O2          __attribute__((optimize("O2")))
#define _O3          __attribute__((optimize("O3")))

#ifndef UNUSED
  #if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)
    #define UNUSED(X) (void)X
  #else
    #define UNUSED(x) ((void)(x))
  #endif
#endif

// Clock speed factors
#if !defined(CYCLES_PER_MICROSECOND) && !defined(__STM32F1__)
  #define CYCLES_PER_MICROSECOND (F_CPU / 1000000UL) // 16 or 20 on AVR
#endif

// Nanoseconds per cycle
#define NANOSECONDS_PER_CYCLE (1000000000.0 / F_CPU)

// Macros to make sprintf_P read from PROGMEM (AVR extension)
#ifdef __AVR__
  #define S_FMT "%S"
#else
  #define S_FMT "%s"
#endif

// Macros to make a string from a macro
#define STRINGIFY_(M) #M
#define STRINGIFY(M) STRINGIFY_(M)

#define A(CODE) " " CODE "\n\t"
#define L(CODE) CODE ":\n\t"

// Macros for bit masks
#undef _BV
#define TEST(n,b) (!!((n)&_BV(b)))
#define SET_BIT_TO(N,B,TF) do{ if (TF) SBI(N,B); else CBI(N,B); }while(0)

#ifndef SBI
  #define SBI(A,B) (A |= (1 << (B)))
#endif

#ifndef CBI
  #define CBI(A,B) (A &= ~(1 << (B)))
#endif

#define _BV32(b) (1UL << (b))
#define TEST32(n,b) !!((n)&_BV32(b))
#define SBI32(n,b) (n |= _BV32(b))
#define CBI32(n,b) (n &= ~_BV32(b))

#define cu(x)      ((x)*(x)*(x))
#define RADIANS(d) ((d)*float(M_PI)/180.0f)
#define DEGREES(r) ((r)*180.0f/float(M_PI))
#define HYPOT2(x,y) (sq(x)+sq(y))

#define CIRCLE_AREA(R) (float(M_PI) * sq(float(R)))
#define CIRCLE_CIRC(R) (2 * float(M_PI) * float(R))

#define SIGN(a) ((a>0)-(a<0))
#define IS_POWER_OF_2(x) ((x) && !((x) & ((x) - 1)))

// Macros to constrain values
#ifdef __cplusplus

  // C++11 solution that is standards compliant.
  template <class V, class N> static inline constexpr void NOLESS(V& v, const N n) {
    if (n > v) v = n;
  }
  template <class V, class N> static inline constexpr void NOMORE(V& v, const N n) {
    if (n < v) v = n;
  }
  template <class V, class N1, class N2> static inline constexpr void LIMIT(V& v, const N1 n1, const N2 n2) {
    if (n1 > v) v = n1;
    else if (n2 < v) v = n2;
  }

#else

  // Using GCC extensions, but Travis GCC version does not like it and gives
  //  "error: statement-expressions are not allowed outside functions nor in template-argument lists"
  #define NOLESS(v, n) \
    do{ \
      __typeof__(n) _n = (n); \
      if (_n > v) v = _n; \
    }while(0)

  #define NOMORE(v, n) \
    do{ \
      __typeof__(n) _n = (n); \
      if (_n < v) v = _n; \
    }while(0)

  #define LIMIT(v, n1, n2) \
    do{ \
      __typeof__(n1) _n1 = (n1); \
      __typeof__(n2) _n2 = (n2); \
      if (_n1 > v) v = _n1; \
      else if (_n2 < v) v = _n2; \
    }while(0)

#endif


#define _ISENA_     ~,1
#define _ISENA_1    ~,1
#define _ISENA_0x1  ~,1
#define _ISENA_true ~,1

#define WITHIN(N,L,H)       ((N) >= (L) && (N) <= (H))
#define NUMERIC(a)          WITHIN(a, '0', '9')
#define DECIMAL(a)          (NUMERIC(a) || a == '.')
#define NUMERIC_SIGNED(a)   (NUMERIC(a) || (a) == '-' || (a) == '+')
#define DECIMAL_SIGNED(a)   (DECIMAL(a) || (a) == '-' || (a) == '+')
#define COUNT(a)            (sizeof(a)/sizeof(*a))
#define ZERO(a)             memset(a,0,sizeof(a))
#define COPY(a,b) do{ \
    static_assert(sizeof(a[0]) == sizeof(b[0]), "COPY: '" STRINGIFY(a) "' and '" STRINGIFY(b) "' types (sizes) don't match!"); \
    memcpy(&a[0],&b[0],_MIN(sizeof(a),sizeof(b))); \
  }while(0)


#define NOOP (void(0))

#define CEILING(x,y) (((x) + (y) - 1) / (y))

#undef ABS
#ifdef __cplusplus
  template <class T> static inline constexpr const T ABS(const T v) { return v >= 0 ? v : -v; }
#else
  #define ABS(a) ({__typeof__(a) _a = (a); _a >= 0 ? _a : -_a;})
#endif

#define UNEAR_ZERO(x) ((x) < 0.000001f)
#define NEAR_ZERO(x) WITHIN(x, -0.000001f, 0.000001f)
#define NEAR(x,y) NEAR_ZERO((x)-(y))

#define RECIPROCAL(x) (NEAR_ZERO(x) ? 0 : (1 / float(x)))
#define FIXFLOAT(f) (f + (f < 0 ? -0.00005f : 0.00005f))

//
// Maths macros that can be overridden by HAL
//
#define ACOS(x)     acosf(x)
#define ATAN2(y, x) atan2f(y, x)
#define POW(x, y)   powf(x, y)
#define SQRT(x)     sqrtf(x)
#define RSQRT(x)    (1.0f / sqrtf(x))
#define CEIL(x)     ceilf(x)
#define FLOOR(x)    floorf(x)
#define LROUND(x)   lroundf(x)
#define FMOD(x, y)  fmodf(x, y)
#define HYPOT(x,y)  SQRT(HYPOT2(x,y))
#define sq(x)       (x*x)

#ifdef TARGET_LPC1768
  #define I2C_ADDRESS(A) ((A) << 1)
#else
  #define I2C_ADDRESS(A) A
#endif

#ifdef __cplusplus

  #ifndef _MINMAX_H_
  #define _MINMAX_H_

    extern "C++" {

      // C++11 solution that is standards compliant. Return type is deduced automatically
      template <class L, class R> static inline constexpr auto _MIN(const L lhs, const R rhs) -> decltype(lhs + rhs) {
        return lhs < rhs ? lhs : rhs;
      }
      template <class L, class R> static inline constexpr auto _MAX(const L lhs, const R rhs) -> decltype(lhs + rhs) {
        return lhs > rhs ? lhs : rhs;
      }
      template<class T, class ... Ts> static inline constexpr const T _MIN(T V, Ts... Vs) { return _MIN(V, _MIN(Vs...)); }
      template<class T, class ... Ts> static inline constexpr const T _MAX(T V, Ts... Vs) { return _MAX(V, _MAX(Vs...)); }

    }

  #endif

#else

  #define MIN_2(a,b)      ((a)<(b)?(a):(b))
  #define MIN_3(a,V...)   MIN_2(a,MIN_2(V))
  #define MIN_4(a,V...)   MIN_2(a,MIN_3(V))
  #define MIN_5(a,V...)   MIN_2(a,MIN_4(V))
  #define MIN_6(a,V...)   MIN_2(a,MIN_5(V))
  #define MIN_7(a,V...)   MIN_2(a,MIN_6(V))
  #define MIN_8(a,V...)   MIN_2(a,MIN_7(V))
  #define MIN_9(a,V...)   MIN_2(a,MIN_8(V))
  #define MIN_10(a,V...)  MIN_2(a,MIN_9(V))
  #define __MIN_N(N,V...) MIN_##N(V)
  #define _MIN_N(N,V...)  __MIN_N(N,V)
  #define _MIN(V...)      _MIN_N(NUM_ARGS(V), V)

  #define MAX_2(a,b)      ((a)>(b)?(a):(b))
  #define MAX_3(a,V...)   MAX_2(a,MAX_2(V))
  #define MAX_4(a,V...)   MAX_2(a,MAX_3(V))
  #define MAX_5(a,V...)   MAX_2(a,MAX_4(V))
  #define MAX_6(a,V...)   MAX_2(a,MAX_5(V))
  #define MAX_7(a,V...)   MAX_2(a,MAX_6(V))
  #define MAX_8(a,V...)   MAX_2(a,MAX_7(V))
  #define MAX_9(a,V...)   MAX_2(a,MAX_8(V))
  #define MAX_10(a,V...)  MAX_2(a,MAX_9(V))
  #define __MAX_N(N,V...) MAX_##N(V)
  #define _MAX_N(N,V...)  __MAX_N(N,V)
  #define _MAX(V...)      _MAX_N(NUM_ARGS(V), V)

#endif

// Macros for adding
#define INC_0 1
#define INC_1 2
#define INC_2 3
#define INC_3 4
#define INC_4 5
#define INC_5 6
#define INC_6 7
#define INC_7 8
#define INC_8 9
#define INCREMENT_(n) INC_##n
#define INCREMENT(n) INCREMENT_(n)

#define ADD0(N)  N
#define ADD1(N)  INCREMENT_(N)
#define ADD2(N)  ADD1(ADD1(N))
#define ADD3(N)  ADD1(ADD2(N))
#define ADD4(N)  ADD2(ADD2(N))
#define ADD5(N)  ADD2(ADD3(N))
#define ADD6(N)  ADD3(ADD3(N))
#define ADD7(N)  ADD3(ADD4(N))
#define ADD8(N)  ADD4(ADD4(N))
#define ADD9(N)  ADD4(ADD5(N))
#define ADD10(N) ADD5(ADD5(N))

// Macros for subtracting
#define DEC_0 0
#define DEC_1 0
#define DEC_2 1
#define DEC_3 2
#define DEC_4 3
#define DEC_5 4
#define DEC_6 5
#define DEC_7 6
#define DEC_8 7
#define DEC_9 8
#define DECREMENT_(n) DEC_##n
#define DECREMENT(n) DECREMENT_(n)

#define SUB0(N)  N
#define SUB1(N)  DECREMENT_(N)
#define SUB2(N)  SUB1(SUB1(N))
#define SUB3(N)  SUB1(SUB2(N))
#define SUB4(N)  SUB2(SUB2(N))
#define SUB5(N)  SUB2(SUB3(N))
#define SUB6(N)  SUB3(SUB3(N))
#define SUB7(N)  SUB3(SUB4(N))
#define SUB8(N)  SUB4(SUB4(N))
#define SUB9(N)  SUB4(SUB5(N))
#define SUB10(N) SUB5(SUB5(N))

// Defer expansion
#define EMPTY()
#define DEFER(M)  M EMPTY()
#define DEFER2(M) M EMPTY EMPTY()()
#define DEFER3(M) M EMPTY EMPTY EMPTY()()()
#define DEFER4(M) M EMPTY EMPTY EMPTY EMPTY()()()()



#define _BV(b) (1UL << (b))

#define EXTRUDERS 1
#define XYZ 3

#define MIN_STEPS_PER_SEGMENT 1
#define MAXIMUM_STEPPER_RATE 5000000
#define F_CPU 16000000
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000)
#define MINIMUM_STEPPER_PULSE 0
#define PULSE_TIMER_NUM 0
#define MINIMUM_PLANNER_SPEED 0.05