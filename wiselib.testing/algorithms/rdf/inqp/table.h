/***************************************************************************
 ** This file is part of the generic algorithm library Wiselib.           **
 ** Copyright (C) 2008,2009 by the Wisebed (www.wisebed.eu) project.      **
 **                                                                       **
 ** The Wiselib is free software: you can redistribute it and/or modify   **
 ** it under the terms of the GNU Lesser General Public License as        **
 ** published by the Free Software Foundation, either version 3 of the    **
 ** License, or (at your option) any later version.                       **
 **                                                                       **
 ** The Wiselib is distributed in the hope that it will be useful,        **
 ** but WITHOUT ANY WARRANTY; without even the implied warranty of        **
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
 ** GNU Lesser General Public License for more details.                   **
 **                                                                       **
 ** You should have received a copy of the GNU Lesser General Public      **
 ** License along with the Wiselib.                                       **
 ** If not, see <http://www.gnu.org/licenses/>.                           **
 ***************************************************************************/

#ifndef TABLE_H
#define TABLE_H

#include <algorithms/rdf/inqp/row.h>
#include <util/pstl/algorithm.h>

namespace wiselib {
	
	/**
	 * @brief
	 * 
	 * Table t;
	 * 
	 * for(Table::iterator it = t.begin(); it != t.end(); ++it) {
	 *    print(*it);
	 * }
	 * 
	 * 
	 * @ingroup
	 * 
	 * @tparam 
	 */
	template<
		typename OsModel_P,
		typename Row_P = Row<OsModel_P>
	>
	class Table {
		
		public:
			typedef OsModel_P OsModel;
			typedef typename OsModel::block_data_t block_data_t;
			typedef typename OsModel::size_t size_type;
			typedef Row_P RowT;
			
			enum { MIN_CAPACITY = 4 };
			
			class iterator {
				public:
					iterator(block_data_t *p, size_type row_size) : p_(p), row_size_(row_size) {
					}
					
					iterator(const iterator& other) {
						*this = other;
					}
					
					/*
					 * operator=()
					 */
					iterator& operator=(const iterator& other) {
						p_ = other.p_;
						row_size_ = other.row_size_;
						return *this;
					}
					
					iterator& operator++() {
						p_ += row_size_;
						return *this;
					}
					
					/*
					iterator& operator+=(int n) {
						p_ += n * row_size_;
						return *this;
					}
					
					RowT& operator[](int n) {
						return *reinterpret_cast<RowT*>(p_ + n * row_size_);
					}
					*/
					
					RowT& operator*() {
						return *reinterpret_cast<RowT*>(p_);
					}
					
					RowT* operator->() { return &operator*(); }
					
					// ??
					/*bool rows_available() {
						// TODO
					}*/
					
					bool operator==(const iterator& other) {
						return (p_ == other.p_) && (row_size_ == other.row_size_);
					}
					
					bool operator!=(const iterator& other) { return !(*this == other); }
					
				//private:
					block_data_t *p_;
					size_type row_size_;
			};
			
			void init(size_type columns) {
				row_size_ = columns * sizeof(typename RowT::Value);
				capacity_ = 0;
				size_ = 0;
				buffer_ = 0;
			}
			
			void insert(const RowT& row) {
				if(size_ >= capacity_) {
					grow();
				}
				memcpy(buffer_ + size_ * row_size_, &row, row_size_);
				size_++;
			}
			
			iterator begin() {
				return iterator(buffer_, row_size_);
			}
			
			iterator end() {
				return iterator(buffer_ + size_ * row_size_, row_size_);
			}
			
			void pack() {
				change_capacity(size_);
			}
			
			RowT& operator[](size_type n) {
				return *reinterpret_cast<RowT*>(
						buffer_ + n * row_size_
				);
			}
			
			void set(size_type i, const RowT& row) {
				memcpy(buffer_ + i * row_size_, &row, row_size_);
			}
			
			size_type size() { return size_; }
			
			void clear() {
				size_ = 0;
				change_capacity(0);
			}
			
			template<typename Compare>
			void sort(Compare& comp) {
				heap_sort(begin(), end(), comp);
			}
			
			void pop_back() {
				if(size_ > 0) {
					size_--;
				}
		 
				if(size_ < (capacity_ / 4)) {
					shrink();
				}
			}
			
		private:
			
			void grow() {
				if(capacity_ < MIN_CAPACITY) {
					change_capacity(MIN_CAPACITY);
				}
				else {
					change_capacity(capacity_ * 2);
				}
			}
			
			void shrink() {
				if(capacity_ < 2 * MIN_CAPACITY) {
					change_capacity(MIN_CAPACITY);
				}
				else {
					change_capacity(capacity_ / 2);
				}
			}
			
			void change_capacity(size_type n) {
				block_data_t *new_buffer = 0;
				if(n > 0) {
					new_buffer = get_allocator().template allocate_array<block_data_t>(n * row_size_).raw();
				}
				if(buffer_) {
					memcpy(new_buffer, buffer_, size_ * row_size_);
					get_allocator().free_array(buffer_);
					buffer_ = 0;
				}
				buffer_ = new_buffer;
				capacity_ = n;
			}
			
			size_type row_size_;
			::uint16_t capacity_;
			::uint16_t size_;
			block_data_t *buffer_;
		
	}; // Table
}

#endif // TABLE_H

