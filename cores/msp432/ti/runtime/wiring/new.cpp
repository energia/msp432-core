#include "new.h"

#if 1
static void * smalloc(size_t size)
{
  for (;;) {
    void *ptr = malloc(size);
    if (ptr) {
        return ptr; /* never return NULL */
    }
  }
}
#else
#define smalloc malloc
#endif

void * operator new(size_t size)
{
    return smalloc(size);
}

void operator delete(void * ptr)
{
  free(ptr);
}

void * operator new[](size_t size)
{
  return smalloc(size);
}

void operator delete[](void * ptr)
{
  free(ptr);
}

int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);};
void __cxa_guard_release (__guard *g) {*(char *)g = 1;};
void __cxa_guard_abort (__guard *) {};

void __cxa_pure_virtual(void) {};

