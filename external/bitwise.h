#ifndef BITWISE_H
#define BITWISE_H

// generate bitmask for len bits at offset i
#define bits_msk(len, i) (((1 << (len)) - 1) << ((len)*(i)))

// basic bitwise operations
#define bits_get(reg, len, i) ((reg)  &  bits_msk((len), (i)))
#define bits_set(reg, len, i) ((reg) |=  bits_msk((len), (i)))
#define bits_flp(reg, len, i) ((reg) ^=  bits_msk((len), (i)))
#define bits_clr(reg, len, i) ((reg) &= ~bits_msk((len), (i)))

// set len bits at offset i to v
#define bits_val(reg, len, i, v) \
	((reg) = ((reg) & ~bits_msk((len), (i))) | ((v) << ((len)*(i))))
//           ^-------- clear bits ---------^   ^--- set value ----^

// single bit operations
#define bit_get(reg, i)    bits_get((reg), 1, (i))
#define bit_set(reg, i)    bits_set((reg), 1, (i))
#define bit_flp(reg, i)    bits_flp((reg), 1, (i))
#define bit_clr(reg, i)    bits_clr((reg), 1, (i))
#define bit_val(reg, i, v) bits_val((reg), 1, (i), (v))

#endif // !BITWISE_H
