#include <stdio.h>
#include <cstdint>
#pragma pack(push, 1)
#define POINTNUM 72
struct V2Header {
	uint32_t magic;  // always 0x6f746c41 in little endian. this will appear as "Alto" in wireshark
	uint32_t nsec;  // nanoseconds
	uint64_t sec;  // seconds
	uint32_t frame_id;  // sequentially incremented ID for every new pointcloud
	uint8_t radar_id;  // which radar this pointcloud came from
	uint8_t mode;  // the mode used for this pointcloud. always 0 for v3/v4/rf6
	uint16_t reserved;
	uint32_t length;  // total size of the pointcloud data in bytes
	uint32_t offset;  // offset in bytes of the data in this packet
			  // the rest of the data following this header is the payload
};

struct V2Point {
	float range;
	float doppler;
	float azi;
	float ele;
	float snr;
};
typedef struct POINTCLOUD {
    V2Header pckHeader;
    V2Point point[POINTNUM];
} POINTCLOUD;
#pragma pack(pop)