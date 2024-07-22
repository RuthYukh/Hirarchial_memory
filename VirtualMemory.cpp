#include "MemoryConstants.h"
#include "PhysicalMemory.h"
#include <cmath>
#include <algorithm>

/** FUNCTION DECLARATIONS */
/**
 * sets frame with 0
 */
void set_zeroes (word_t frame_idx);
/**
 * performs dfs in the hierarchical tree and updates possible next frames
 * (option 1 and 3)
 */
void find_frame_dfs (word_t cur_frame, word_t *max_frame, word_t cur_depth,
                     word_t frame_entered, word_t *found_zero,
                     word_t *zero_parent_i, word_t *zero_parent_pointer,
                     word_t parent, uint64_t dest_page, uint64_t cur_page,
                     uint64_t *max_dist_page, uint64_t *max_dist,
                     word_t *frame_of_max_page,
                     word_t *max_dist_parent_pointer,
                     word_t *max_dist_parent_i, word_t prev_i);
/**
 * finds the max cyclic dist
 */
void cyclic_dist (uint64_t dest_page, uint64_t cur_page,
                  uint64_t *max_dist_page, uint64_t *max_dist,
                  word_t cur_frame, word_t *frame_of_max_page,
                  word_t *max_dist_parent_pointer, word_t *max_dist_parent_i,
                  word_t max_dist_parent, word_t i);
/**
 * translates the virtual address to physical frame
 */
word_t translate_address (uint64_t virtualAddress);

void VMinitialize()
{
  set_zeroes (0);
}

void set_zeroes (word_t frame_idx)
{
  for (int i=0; i < PAGE_SIZE; i++)
  {
    PMwrite ((frame_idx*PAGE_SIZE) + i, 0);
  }
}

int VMread(uint64_t virtualAddress, word_t* value)
{
  if (virtualAddress >= VIRTUAL_MEMORY_SIZE)
  {
    return 0;
  }
  word_t cur_frame = translate_address (virtualAddress);

  uint64_t pre_offset = (1LL << OFFSET_WIDTH)-1;
  uint64_t offset = pre_offset & virtualAddress;
  PMread((cur_frame*PAGE_SIZE)+offset, value);
  return 1;
}

void find_frame_dfs (word_t cur_frame, word_t *max_frame, word_t cur_depth,
                     word_t frame_entered, word_t *found_zero,
                     word_t *zero_parent_i, word_t *zero_parent_pointer,
                     word_t parent, uint64_t dest_page, uint64_t cur_page,
                     uint64_t *max_dist_page, uint64_t *max_dist,
                     word_t *frame_of_max_page,
                     word_t *max_dist_parent_pointer,
                     word_t *max_dist_parent_i, word_t prev_i)
{
  if (cur_depth == TABLES_DEPTH)
  {
    if (cur_frame != frame_entered)
    {
      cyclic_dist (dest_page, cur_page, max_dist_page, max_dist, cur_frame,
                   frame_of_max_page, max_dist_parent_pointer,
                   max_dist_parent_i, parent, prev_i);
    }
    return;
  }
  bool current_all_zero = true;
  for (int i = 0; i < PAGE_SIZE; i++)
  {
    word_t line;
    PMread((cur_frame * PAGE_SIZE) + i, &line);
    if (line != 0)
    {
      current_all_zero = false;
      if (line > *max_frame)
      {
        *max_frame = line;
      }
      uint64_t next_address = (cur_page << OFFSET_WIDTH) +(uint64_t) i;
      find_frame_dfs (line, max_frame,
                      cur_depth + 1, frame_entered, found_zero, zero_parent_i,
                      zero_parent_pointer, cur_frame, dest_page, next_address,
                      max_dist_page, max_dist, frame_of_max_page,
                      max_dist_parent_pointer, max_dist_parent_i, i);
    }
  }
  if (current_all_zero && (cur_frame != frame_entered))
  {
    *found_zero = cur_frame;
    *zero_parent_i = prev_i;
    *zero_parent_pointer = parent;
    return;
  }
}

int VMwrite(uint64_t virtualAddress, word_t value)
{
  if (virtualAddress >= VIRTUAL_MEMORY_SIZE)
  {
    return 0;
  }
  word_t cur_frame = translate_address (virtualAddress);
  uint64_t pre_offset = (1LL << OFFSET_WIDTH)-1;
  uint64_t offset = pre_offset & virtualAddress;
  PMwrite ((cur_frame*PAGE_SIZE)+offset, value);
  return 1;

}
void cyclic_dist (uint64_t dest_page, uint64_t cur_page,
                  uint64_t *max_dist_page, uint64_t *max_dist,
                  word_t cur_frame, word_t *frame_of_max_page,
                  word_t *max_dist_parent_pointer, word_t *max_dist_parent_i,
                  word_t max_dist_parent, word_t i)
{
  uint64_t dist = (dest_page > cur_page) ? (dest_page - cur_page) : (cur_page - dest_page);
  uint64_t cur_min_dist = std::min(static_cast<uint64_t> NUM_PAGES-dist,dist);
  if (cur_min_dist > *max_dist) {
    *max_dist = cur_min_dist;
    *max_dist_page = cur_page;
    *frame_of_max_page = cur_frame;
    *max_dist_parent_pointer = max_dist_parent;
    *max_dist_parent_i = i;
  }
}

word_t translate_address (uint64_t virtualAddress)
{
  word_t cur_frame = 0;
  word_t next_frame;
  uint64_t page_num = virtualAddress >> OFFSET_WIDTH;
  for (int i=0; i<TABLES_DEPTH; i++) {
    uint64_t pre_p_i = (1LL << OFFSET_WIDTH) -1;
    word_t shift_factor_i = (TABLES_DEPTH-i)*OFFSET_WIDTH;
    uint64_t p_i = (virtualAddress >> shift_factor_i) & pre_p_i;
    PMread ((cur_frame*PAGE_SIZE)+p_i, &next_frame);
    if (next_frame == 0) {
      word_t max_frame = 0;
      word_t found_zero = -1;
      word_t zero_parent_i = 0;
      word_t zero_parent_node = 0;
      word_t max_dist_parent_node = 0;
      word_t max_dist_parent_i = 0;
      word_t frame_of_max_page = 0;
      uint64_t max_dist_page = -1;
      uint64_t max_dist = 0;
      find_frame_dfs (0, &max_frame,
                      0, cur_frame, &found_zero, &zero_parent_i,
                      &zero_parent_node, -1, page_num, 0, &max_dist_page,
                      &max_dist, &frame_of_max_page, &max_dist_parent_node,
                      &max_dist_parent_i, -1);
      if (found_zero == -1) {
        if (max_frame + 1 < NUM_FRAMES) {
          next_frame = max_frame+1;
        }
        else {
          PMevict (frame_of_max_page, max_dist_page);
          PMwrite ((max_dist_parent_node*PAGE_SIZE) + max_dist_parent_i, 0);
          next_frame = frame_of_max_page;
        }
      }
      else {
        next_frame = found_zero;
        PMwrite ((zero_parent_node * PAGE_SIZE) + zero_parent_i, 0);
      }
      if (i != TABLES_DEPTH-1) {
        set_zeroes (next_frame);
      }
      if (i == TABLES_DEPTH-1) {
        PMrestore (next_frame, page_num);
      }
      PMwrite ((cur_frame*PAGE_SIZE) + p_i, next_frame);
    }
    cur_frame = next_frame;
  }
  return cur_frame;
}