/*
 * virtuafunctionfix.h
 *
 *  Created on: Mar 26, 2011
 *      Author: foley
 */

#ifndef VIRTUALFUNCTIONFIX_H_
#define VIRTUALFUNCTIONFIX_H_
#include <inttypes.h>
#include <WProgram.h>

extern "C" void __cxa_pure_virtual();
__extension__ typedef int __guard __attribute__((mode (__DI__)));
void * operator new(size_t size);
void operator delete(void * ptr);
void * operator new[](size_t size);
void operator delete[](void * ptr);
int __cxa_guard_acquire(__guard *g);
void __cxa_guard_release (__guard *g);
void __cxa_guard_abort (__guard *);
#endif /* VIRTUALFUNCTIONFIX_H_ */
