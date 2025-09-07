/*
    Copyright 2024 Picovoice Inc.

    You may not use this file except in compliance with the license. A copy of the license is located in the "LICENSE"
    file accompanying this source.

    Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
    an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
    specific language governing permissions and limitations under the License.
*/

#ifndef PV_CIRCULAR_BUFFER_H
#define PV_CIRCULAR_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

/**
* Forward declaration of pv_circular_buffer object. It handles reading and writing to a circular buffer.
*/
typedef struct pv_circular_buffer pv_circular_buffer_t;

/**
* Status codes.
*/
typedef enum {
    PV_CIRCULAR_BUFFER_STATUS_SUCCESS = 0,
    PV_CIRCULAR_BUFFER_STATUS_OUT_OF_MEMORY,
    PV_CIRCULAR_BUFFER_STATUS_INVALID_ARGUMENT,
    PV_CIRCULAR_BUFFER_STATUS_WRITE_OVERFLOW,
} pv_circular_buffer_status_t;

/**
* Constructor for pv_circular_buffer object.
*
* @param element_count Capacity of the buffer to read and write.
* @param element_size Size of each element in the buffer.
* @param object[out] Circular buffer object.
* @return Status Code. Returns PV_CIRCULAR_BUFFER_STATUS_OUT_OF_MEMORY or PV_CIRCULAR_BUFFER_STATUS_INVALID_ARGUMENT
* on failure.
*/
pv_circular_buffer_status_t pv_circular_buffer_init(
        int32_t element_count,
        int32_t element_size,
        pv_circular_buffer_t **object);

/**
* Destructor for pv_circular_buffer object.
*
* @param object Circular buffer object.
*/
void pv_circular_buffer_delete(pv_circular_buffer_t *object);

/**
* Reads and copies the elements to the provided buffer.
*
* @param object Circular buffer object.
* @param buffer[out] A pointer to a pre-allocated buffer to receive the copied data.
* @param buffer_length The maximum number of elements that can be copied into `buffer`.
* @param read_length[out] Actual number of elements read.
* @return Status Code. Returns PV_CIRCULAR_BUFFER_STATUS_INVALID_ARGUMENT on failure.
*/
pv_circular_buffer_status_t pv_circular_buffer_read(
        pv_circular_buffer_t *object,
        void *buffer,
        int32_t buffer_length,
        int32_t *read_length);

/**
* Writes and copies the elements of `buffer` to the object's buffer. Does not write frames if the buffer
* is full and returns PV_CIRCULAR_BUFFER_STATUS_WRITE_OVERFLOW which is not a failure.
*
* @param object Circular buffer object.
* @param buffer A pointer to copy its elements to the object's buffer.
* @param buffer_length The amount of elements to copy.
* @return Status Code. Returns PV_CIRCULAR_BUFFER_STATUS_INVALID_ARGUMENT on failure.
*/
pv_circular_buffer_status_t pv_circular_buffer_write(
        pv_circular_buffer_t *object,
        const void *buffer,
        int32_t buffer_length);

/**
* Gets the current size of the object's buffer.
*
* @param object Circular buffer object.
* @param count[out] The current size of the buffer.
* @return Status Code. Returns PV_CIRCULAR_BUFFER_STATUS_INVALID_ARGUMENT on failure.
*/
pv_circular_buffer_status_t pv_circular_buffer_get_count(pv_circular_buffer_t *object, int32_t *count);

/**
* Reset the buffer pointers to start.
*
* @param object Circular buffer object.
*/
void pv_circular_buffer_reset(pv_circular_buffer_t *object);

/**
* Provides string representations of status codes.
*
* @param status Status code.
* @return String representation.
*/
const char *pv_circular_buffer_status_to_string(pv_circular_buffer_status_t status);

#endif //PV_CIRCULAR_BUFFER_H