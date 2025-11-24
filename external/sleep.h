#ifndef SLEEP_H
#define SLEEP_H

// sleep for seconds
#define sleep(t) __sleep((t), 1)

// sleep for milliseconds
#define sleep_ms(t) __sleep((t), 1000)

// actual sleep function (use the macros)
void __sleep(unsigned int t, unsigned int s);

#endif // !SLEEP_H
