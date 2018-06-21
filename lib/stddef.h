#ifndef STDDEF_H
#define STDDEF_H

typedef __SIZE_TYPE__		size_t;
#define NULL			((void *)0)

#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)

typedef int	ptrdiff_t;

#endif /* STDDEF_H */
