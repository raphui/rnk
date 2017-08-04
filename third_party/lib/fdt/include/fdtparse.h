/*
 * Copyright (C) 2014 F4OS Authors
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef DEV_FDTPARSE_H_INCLUDED
#define DEV_FDTPARSE_H_INCLUDED

/* Additional helper functions for parsing FDT */

#include "libfdt.h"

struct fdt_gpio {
    int gpio;
    int flags;
};

/**
 * Get the global FDT blob
 *
 * @returns pointer to the device FDT blob
 */
const void *fdtparse_get_blob(void);

/**
 * Retreive the offset of the node referenced by a given alias
 *
 * @param fdt   pointer to the device tree blob
 * @param name  name of the alias to look up
 * @returns:
 *    structure block offset of the node with the requested alias (>=0),
 *    on success, on error:
 *    -FDT_ERR_NOTFOUND, if the requested alias or /aliases does not exist
 *    -FDT_ERR_BADMAGIC,
 *    -FDT_ERR_BADVERSION,
 *    -FDT_ERR_BADSTATE,
 *    -FDT_ERR_BADSTRUCTURE,
 *    -FDT_ERR_TRUNCATED, standard meanings.
 */
int fdtparse_alias_offset( const char *name );

/**
 * Extract 32-bit integer from propery
 *
 * @param fdt   pointer to the device tree blob
 * @param offset    offset of node containing desired propery
 * @param name  string name of register property
 * @param val   pointer to int to place result in
 * @returns 0 on success, on error:
 *    -FDT_ERR_NOTFOUND, if property is not found, or too small
 *    -FDT_ERR_BADMAGIC,
 *    -FDT_ERR_BADVERSION,
 *    -FDT_ERR_BADSTATE,
 *    -FDT_ERR_BADSTRUCTURE,
 *    -FDT_ERR_TRUNCATED, standard meanings.
 */
int fdtparse_get_int( int offset, const char *name, int *val );

/**
 * Extract register 32-bit address from 'regs' property
 *
 * Note: the address is expected to be a 32-bit address; 1 address-cell.
 *
 * @param fdt   pointer to the device tree blob
 * @param offset    offset of node containing desired propery
 * @param name  string name of register property
 * @returns address stored in property, NULL on error
 */
void *_fdtparse_get_addr32( int offset, const char *name, fdt32_t *poff ) ;

void *fdtparse_get_addr32( int offset, const char *name );

/**
 * Extract GPIO information from GPIO cell property
 *
 * GPIO cells have the form <&gpio_device gpio_num gpio_flags>.  This function
 * extracts that information into a struct fdt_gpio.
 *
 * @param fdt   pointer to the device tree blob
 * @param offset    offset of node containing desired propery
 * @param name  string name of gpio property
 * @param gpio  pointer to struct fdt_gpio to return value in
 * @returns 0 on success,
 *    -FDT_ERR_BADLAYOUT, if the cell property is not 3 values in length
 *    -FDT_ERR_NOTFOUND, node does not have named property
 *    -FDT_ERR_BADOFFSET, offset did not point to FDT_BEGIN_NODE tag
 *    -FDT_ERR_BADMAGIC,
 *    -FDT_ERR_BADVERSION,
 *    -FDT_ERR_BADSTATE,
 *    -FDT_ERR_BADSTRUCTURE,
 *    -FDT_ERR_TRUNCATED, standard meanings
 */
int fdtparse_get_gpio( int offset, const char *name, struct fdt_gpio *gpio);

/**
 * Extract GPIO information from multiple GPIO cells
 *
 * GPIO cells have the form <&gpio_device gpio_num gpio_flags>.  This function
 * extracts that information from a list of cells into an array of
 * struct fdt_gpio.
 *
 * @param fdt   pointer to the device tree blob
 * @param offset    offset of node containing desired propery
 * @param name  string name of gpio property
 * @param gpio  array of struct fdt_gpio to return values in
 * @param max   maximum number of GPIOs to return (size of gpio array)
 * @returns number of gpios read on success,
 *    -FDT_ERR_BADLAYOUT, if max number would be exceeded
 *    -FDT_ERR_NOTFOUND, node does not have named property
 *    -FDT_ERR_BADOFFSET, offset did not point to FDT_BEGIN_NODE tag
 *    -FDT_ERR_BADMAGIC,
 *    -FDT_ERR_BADVERSION,
 *    -FDT_ERR_BADSTATE,
 *    -FDT_ERR_BADSTRUCTURE,
 *    -FDT_ERR_TRUNCATED, standard meanings
 */
int fdtparse_get_gpios( int offset, const char *name, struct fdt_gpio *gpio, int max);

/**
 * Get full path to a node
 *
 * Compute the full path of a node at offset, recording that path in the
 * buffer returned.  This buffer is malloc()'d, and must be free()'d when
 * no longer needed.
 *
 * NOTE: This function is expensive, as it must scan the device tree
 * structure from the start to nodeoffset, possibly multiple times,
 * depending on the size of the path.
 *
 * @param fdt  pointer to the device tree blob
 * @param offset    offset of node to get full path of
 * @returns pointer to NUL-terminated buffer containing full path, or
 *  NULL on error
 */
char *fdtparse_get_path( int offset);

/**
 * Find the interrupt-parent node
 *
 * Uses the "interrupt-parent" property to find the offset of the interrupt
 * parent of a given node.  If the property is not found, parent nodes
 * will be searched for the propery.
 *
 * @param fdt   pointer to the device tree blob
 * @param nodeoffset    node to find interrupt parent of
 * @returns,
 *    structure block offset of the located node (>= 0), on success
 *    -FDT_ERR_NOTFOUND, no interrupt-parent property found,
 *    -FDT_ERR_BADPHANDLE, interrupt-parent property does not contain a phandle,
 *    -FDT_ERR_BADMAGIC,
 *    -FDT_ERR_BADVERSION,
 *    -FDT_ERR_BADSTATE,
 *    -FDT_ERR_BADSTRUCTURE, standard meanings
 */
int fdtparse_get_interrupt_parent( int nodeoffset);

/**
 * Get next string in stringlist
 *
 * Walks the stringlist, finding the next string before the end.
 *
 * @param strlist   pointer to the start of the stringlist
 * @param curr      pointer to current string in stringlist
 * @param listlen   length of entire stringlist
 * @returns pointer to next string in stringlist, or NULL if none
 */
const char *fdtparse_stringlist_next(const char *strlist, const char *curr,
                                     int listlen);


int fdtparse_get_u32_array(int offset, const char *name, unsigned int *out_values, int size);

int fdtparse_get_u16_array(int offset, const char *name, unsigned short *out_values, int size);

int fdtparse_get_u8_array(int offset, const char *name, unsigned char *out_values, int size);
#endif
