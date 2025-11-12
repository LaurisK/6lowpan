/*
 * Copyright (c) 2013, Swedish Institute of Computer Science
 * Copyright (c) 2010, Vrije Universiteit Brussel
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 * Authors: Simon Duquennoy <simonduq@sics.se>
 *          Joris Borms <joris.borms@vub.ac.be>
 */

#include <stddef.h>
#include <string.h>
#include "nbr-table.h"

#define NBR_DEBUG 0
#if NBR_DEBUG
#include "App/Time/time.h"
#include "cmsis_os.h"
#include "../routing/rpl-lite/rpl-nbr-policy.h"

static uint8_t initialized = 0;
static TimerHandle_t dbgTimer;
#else
#define PRINTF(...)
#endif

/* List of link-layer addresses of the neighbors, used as key in the tables */
typedef struct nbr_table_key {
  struct nbr_table_key *next;
  linkaddr_t lladdr;
} nbr_table_key_t;

/* For each neighbor, a map of the tables that use the neighbor.
 * As we are using uint8_t, we have a maximum of 8 tables in the system */
static uint8_t used_map[NBR_TABLE_MAX_NEIGHBORS];
/* For each neighbor, a map of the tables that lock the neighbor */
static uint8_t locked_map[NBR_TABLE_MAX_NEIGHBORS];
/* The maximum number of tables */
#define MAX_NUM_TABLES 8
/* A list of pointers to tables in use */
static struct nbr_table *all_tables[MAX_NUM_TABLES];
/* The current number of tables */
static unsigned num_tables;

/* The neighbor address table */
static nbr_table_key_t neighborAddr[NBR_TABLE_MAX_NEIGHBORS] = {0};
static nbr_table_key_t *keyListHead = NULL, *keyListTail = NULL;

/*---------------------------------------------------------------------------*/
/* Get a key from a neighbor index */
static nbr_table_key_t* key_from_index(int index) {
  return (index != -1) ? &neighborAddr[index] : NULL;
}
/*---------------------------------------------------------------------------*/
/* Get an item from its neighbor index */
static nbr_table_item_t * item_from_index(nbr_table_t *table, int index) {
  return table != NULL && index != -1 ? (char *)table->data + index * table->item_size : NULL;
}
/*---------------------------------------------------------------------------*/
/* Get the neighbor index of an item */
static int index_from_key(nbr_table_key_t *key) {
  return (key != NULL) ? (key - neighborAddr) : -1;
}
/*---------------------------------------------------------------------------*/
/* Get the neighbor index of an item */
static int
index_from_item(nbr_table_t *table, const nbr_table_item_t *item)
{
  return table != NULL && item != NULL ? ((int)((char *)item - (char *)table->data)) / table->item_size : -1;
}
/*---------------------------------------------------------------------------*/
/* Get an item from its key */
static nbr_table_item_t* item_from_key(nbr_table_t *table, nbr_table_key_t *key) {
  return item_from_index(table, index_from_key(key));
}
/*---------------------------------------------------------------------------*/
/* Get the key af an item */
static nbr_table_key_t* key_from_item(nbr_table_t *table, const nbr_table_item_t *item) {
  return key_from_index(index_from_item(table, item));
}
/*---------------------------------------------------------------------------*/
/* Get the index of a neighbor from its link-layer address */
static int index_from_lladdr(const linkaddr_t *lladdr) {
  nbr_table_key_t *key;
  /* Allow lladdr-free insertion, useful e.g. for IPv6 ND.
   * Only one such entry is possible at a time, indexed by linkaddr_null. */
  if(lladdr == NULL) {
    lladdr = &linkaddr_null;
  }
  key = keyListHead;
  while(key != NULL) {
    if(lladdr && linkaddr_cmp(lladdr, &key->lladdr)) {
      return index_from_key(key);
    }
    key = key->next;
  }
  return -1;
}
/*---------------------------------------------------------------------------*/
/* Get bit from "used" or "locked" bitmap */
static int
nbr_get_bit(uint8_t *bitmap, nbr_table_t *table, nbr_table_item_t *item)
{
  int item_index = index_from_item(table, item);
  if(table != NULL && item_index != -1) {
    return (bitmap[item_index] & (1 << table->index)) != 0;
  } else {
    return 0;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Set bit in "used" or "locked" bitmap */
static int nbr_set_bit(uint8_t *bitmap, nbr_table_t *table, nbr_table_item_t *item, int value) {
  int item_index = index_from_item(table, item);

  if(table != NULL && item_index != -1) {
    if(value) {
      bitmap[item_index] |= 1 << table->index;
    } else {
      bitmap[item_index] &= ~(1 << table->index);
    }
    return 1;
  } else {
    return 0;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static void remove_key(nbr_table_key_t *least_used_key) {
  int i;
  nbr_table_key_t *walker = keyListHead, *follower = NULL;
  for(i = 0; i < MAX_NUM_TABLES; i++) {
    if(all_tables[i] != NULL && all_tables[i]->callback != NULL) {
      /* Call table callback for each table that uses this item */
      nbr_table_item_t *removed_item = item_from_key(all_tables[i], least_used_key);
      if(nbr_get_bit(used_map, all_tables[i], removed_item) == 1) {
        all_tables[i]->callback(removed_item);
      }
    }
  }
  /* Empty used map */
  used_map[index_from_key(least_used_key)] = 0;
  /* Remove neighbor from list */
  while (NULL != walker) {
	  if (least_used_key == walker) {
		  if (NULL != follower) {
			  follower->next = walker->next;
		  } else {
			  keyListHead = walker->next;
		  }
		  walker->next = NULL;
		  break;
	  }
	  follower = walker;
	  walker = walker->next;
  }
}
/*---------------------------------------------------------------------------*/
static nbr_table_key_t * nbr_table_allocate(nbr_table_reason_t reason, void *data) {
  static uint8_t unusedKeyPos = 0;
  nbr_table_key_t *key;
  int least_used_count = 0;
  nbr_table_key_t *least_used_key = NULL;

  if(NBR_TABLE_MAX_NEIGHBORS > unusedKeyPos) {
	key = &neighborAddr[unusedKeyPos];
	unusedKeyPos++;
    return key;
  } else {
#ifdef NBR_TABLE_FIND_REMOVABLE
    const linkaddr_t *lladdr;
    lladdr = rpl_nbr_policy_find_removable(reason, data);
    if(lladdr == NULL) {
      /* Nothing found that can be deleted - return NULL to indicate failure */
      //return NULL;
    } else {
      /* used least_used_key to indicate what is the least useful entry */
      int index;
      int locked = 0;
      if((index = index_from_lladdr(lladdr)) != -1) {
        least_used_key = key_from_index(index);
        locked = locked_map[index];
      }
      /* Allow delete of locked item? */
      if(least_used_key != NULL && locked) {
        locked_map[index] = 0;
      }
    }
#endif /* NBR_TABLE_FIND_REMOVABLE */

    if(least_used_key == NULL) {
      /* No more space, try to free a neighbor.
       * The replacement policy is the following: remove neighbor that is:
       * (1) not locked
       * (2) used by fewest tables
       * (3) oldest (the list is ordered by insertion time)
       * */
      /* Get item from first key */
      key = keyListHead;
      while(key != NULL) {
        int item_index = index_from_key(key);
        int locked = locked_map[item_index];
        /* Never delete a locked item */
        if(!locked) {
          int used = used_map[item_index];
          int used_count = 0;
          /* Count how many tables are using this item */
          while(used != 0) {
            if((used & 1) == 1) {
              used_count++;
            }
            used >>= 1;
          }
          /* Find least used item */
          if((least_used_key == NULL) || (used_count < least_used_count)) {
            least_used_key = key;
            least_used_count = used_count;
            if(used_count == 0) { /* We won't find any least used item */
              break;
            }
          }
        }
        key = key->next;
      }
    }

    if(least_used_key == NULL) {
      /* We haven't found any unlocked item, allocation fails */
      return NULL;
    } else {
      /* Reuse least used item */
      remove_key(least_used_key);
      return least_used_key;
    }
  }
}
/*---------------------------------------------------------------------------*/
#if NBR_DEBUG
static void printNbrTable(TimerHandle_t periodicTim)
{
  int i, j;
  /* Printout all neighbors and which tables they are used in */
  TRice("msg:NBR TABLE:");
  for(i = 0; i < NBR_TABLE_MAX_NEIGHBORS; i++) {
    if(used_map[i] > 0) {
      TRice("msg:\n % 2d", i);
      TRiceS("msg: %s ", (char*)linkaddr_printAddr(&neighborAddr[i].lladdr));
      for(j = 0; j < num_tables; j++) {
          TRiceS(" %s", (char*)all_tables[j]->tableName);
          if (0 != (used_map[i] & (1 << j))) {
        	  TRice("|used");
          }
          if (0 != (locked_map[i] & (1 << j))) {
        	  TRice("|locked");
          }
      }
    }
  }
  TRice("\n");
}
#endif
/*---------------------------------------------------------------------------*/
/* Register a new neighbor table. To be used at initialization by modules
 * using a neighbor table */
int nbr_table_register(const char *tblName, nbr_table_t *table, nbr_table_callback *callback, uint8_t layer) {
#if NBR_DEBUG
  if(!initialized) {
    initialized = 1;
    /* schedule a debug printout per minute */
    printNbrTable(dbgTimer);
    dbgTimer = xTimerCreate("6lowpan-nbr-debugTimer", pdMS_TO_TICKS(SECONDS_IN_MINUTE * 1000), pdTRUE, 0, printNbrTable);
    xTimerStart(dbgTimer, 0);
  }
#endif

  if(nbr_table_is_registered(table)) {
    /* Table already registered, just update callback */
    table->callback = callback;
#if NBR_DEBUG
    TRiceS("msg:Neighbor register '%s' table callback updated.\n", (char*)tblName);
#endif
    return 1;
  }

  if(num_tables < MAX_NUM_TABLES) {
    table->index = num_tables++;
    table->callback = callback;
    all_tables[table->index] = table;
    table->tableName = tblName;
    table->layer = layer;
#if NBR_DEBUG
    TRiceS("msg:Neighbor register '%s' table", (char*)tblName);
    TRice("msg: at idx(%d)\n", table->index);
#endif
    return 1;
  } else {
    /* Maximum number of tables exceeded */
#if NBR_DEBUG
    TRice("err:Neighbor register - out of tables\n");
#endif
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
/* Test whether a specified table has been registered or not */
int nbr_table_is_registered(nbr_table_t *table) {
  if(table != NULL && table->index >= 0 && table->index < MAX_NUM_TABLES && all_tables[table->index] == table) {
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Returns the first item of the current table */
nbr_table_item_t * nbr_table_head(nbr_table_t *table) {
  /* Get item from first key */
  nbr_table_item_t *item = item_from_key(table, keyListHead);
  /* Item is the first neighbor, now check is it is in the current table */
  if(nbr_get_bit(used_map, table, item)) {
    return item;
  } else {
    return (NULL != item) ? nbr_table_next(table, item) : (NULL);
  }
}
/*---------------------------------------------------------------------------*/
/* Iterates over the current table */
nbr_table_item_t * nbr_table_next(nbr_table_t *table, nbr_table_item_t *item) {
  do {
	nbr_table_key_t *key = key_from_item(table, item);
    key = key->next;
    /* Loop until the next item is in the current table */
    item = item_from_key(table, key);
  } while(item && !nbr_get_bit(used_map, table, item));
  return item;
}
/*---------------------------------------------------------------------------*/
/* Add a neighbor indexed with its link-layer address */
nbr_table_item_t * nbr_table_add_lladdr(nbr_table_t *table, const linkaddr_t *lladdr, nbr_table_reason_t reason, void *data) {
  int index;
  nbr_table_item_t *item;
  nbr_table_key_t *key;

#if NBR_DEBUG
  TRiceS("msg:Neighbor add %s\n", (char*)linkaddr_printAddr(lladdr));
  printNbrTable(dbgTimer);
#endif
  if(table == NULL) {
#if NBR_DEBUG
	TRice("err:table is NULL\n");
#endif
    return NULL;
  }

  /* Allow lladdr-free insertion, useful e.g. for IPv6 ND.
   * Only one such entry is possible at a time, indexed by linkaddr_null. */
  if(lladdr == NULL) {
    lladdr = &linkaddr_null;
  }
  index = index_from_lladdr(lladdr);
  if(-1 == index) {
     /* Neighbor not yet in table, let's try to allocate one */
    key = nbr_table_allocate(reason, data);

    /* No space available for new entry */
    if(key == NULL) {
#if NBR_DEBUG
	TRice("err:No space available for new entry\n");
#endif
      return NULL;
    }

    /* Add neighbor to list */
    key->next = NULL;
    if (NULL != keyListTail) {
    	keyListTail->next = key;
    } else {
    	keyListHead = key;
    }
    keyListTail = key;

    /* Get index from newly allocated neighbor */
    index = index_from_key(key);

  } else {
	key = key_from_index(index);
  }
  /* Set link-layer address */
  linkaddr_copy(&key->lladdr, lladdr);

  /* Get item in the current table */
  item = item_from_index(table, index);

  /* Initialize item data and set "used" bit */
  memset(item, 0, table->item_size);
  nbr_set_bit(used_map, table, item, 1);

#if NBR_DEBUG
  TRice("msg:Neighbor add to %d\n", index);
  printNbrTable(dbgTimer);
#endif
  return item;
}
/*---------------------------------------------------------------------------*/
/* Get an item from its link-layer address */
void *nbr_table_get_from_lladdr(nbr_table_t *table, const linkaddr_t *lladdr) {
  void *item = item_from_index(table, index_from_lladdr(lladdr));
  return nbr_get_bit(used_map, table, item) ? item : NULL;
}
/*---------------------------------------------------------------------------*/
/* Removes a neighbor from the current table (unset "used" bit) */
int nbr_table_remove(nbr_table_t *table, void *item) {
  int ret = nbr_set_bit(used_map, table, item, 0);
  int index = index_from_item(table, item);
  uint8_t tbl = num_tables;
#if NBR_DEBUG
  TRice("msg:Neighbor remove %d\n", index);
#endif
  nbr_set_bit(locked_map, table, item, 0);
  memset(item, 0, table->item_size);
  while (tbl) {
	tbl--;
	if ((NULL != all_tables[tbl]) && (all_tables[tbl]->layer > table->layer) && ((-1) < index) && (used_map[index] & (1 << all_tables[tbl]->index))) {
	  if (NULL != all_tables[tbl]->callback) {
		all_tables[tbl]->callback(item_from_index(all_tables[tbl], index));
	  } else {
		used_map[index] &= ~(1 << all_tables[tbl]->index);
		locked_map[index] &= ~(1 << all_tables[tbl]->index);
		memset(item_from_index(all_tables[tbl], index), 0, all_tables[tbl]->item_size);
	  }
	}
  }
  return ret;
}
/*---------------------------------------------------------------------------*/
/* Lock a neighbor for the current table (set "locked" bit) */
int nbr_table_lock(nbr_table_t *table, void *item) {
#if NBR_DEBUG
  int i = index_from_item(table, item);
  TRice("msg:Neighbor lock %d\n", i);
#endif
  return nbr_set_bit(locked_map, table, item, 1);
}
/*---------------------------------------------------------------------------*/
/* Release the lock on a neighbor for the current table (unset "locked" bit) */
int nbr_table_unlock(nbr_table_t *table, void *item) {
#if NBR_DEBUG
  int i = index_from_item(table, item);
  TRice("msg:Neighbor unlock %d\n", i);
#endif
  return nbr_set_bit(locked_map, table, item, 0);
}
/*---------------------------------------------------------------------------*/
/* Get link-layer address of an item */
linkaddr_t * nbr_table_get_lladdr(nbr_table_t *table, const void *item) {
  nbr_table_key_t *key = key_from_item(table, item);
  return key != NULL ? &key->lladdr : NULL;
}
