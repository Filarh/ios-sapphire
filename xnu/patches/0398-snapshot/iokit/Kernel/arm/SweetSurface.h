/*
 * Built-in IOSurface for SWEET (see SweetSurface.cpp).
 *
 * Shared with SweetFramebuffer.cpp: the framebuffer looks surfaces up by
 * id when a swap names them and publishes the panel memory itself as the
 * default framebuffer surface.
 */
#ifndef _SWEET_SURFACE_H
#define _SWEET_SURFACE_H

#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <libkern/c++/OSObject.h>
#include <mach/mach_types.h>

#define SWEET_SURFACE_MAX_PLANES        20

struct sweet_surface_plane {
	uint32_t width, height, bytesPerRow, offset, size;
	uint16_t bytesPerElement;
	uint8_t  elementWidth, elementHeight;
};

/* SweetSurface objects alive (+1 at init, -1 at free) and bytes of buffer surfaces, for the 5 s log. */
extern "C" void sweet_surface_counted(int delta);
extern "C" void sweet_surface_census(uint32_t *objects, uint64_t *wired);

class SweetSurface : public OSObject {
	OSDeclareDefaultStructors(SweetSurface);

public:
	uint32_t id;
	uint32_t width, height, bytesPerRow, offset, pixelFormat, allocSize;
	uint16_t bytesPerElement;
	uint8_t  elementWidth, elementHeight;
	uint32_t cacheMode, planeCount, parentID;
	struct sweet_surface_plane planes[SWEET_SURFACE_MAX_PLANES];

	IOMemoryDescriptor       *memory;       /* pixels (buffer or physical) */
	IOOptionBits              mapCache;     /* cache mode for user mappings */
	IOBufferMemoryDescriptor *state;        /* shared state page, read-only for users */
	IOMemoryMap              *kernelMap;    /* lazily created for kernel copies */
	uint64_t                  scanoutPhys;  /* topaz: physically contiguous scanout slot, or 0 */
	int                       scanoutSlot;  /* pool slot, -1 when not from the pool */
	OSDictionary             *values;       /* IOSurfaceSetValue & co. */
	uint32_t                  purgeable;    /* IOSurfaceSetPurgeable state */
	volatile uint32_t         gpuSeed;      /* sapphire 0151: the seed when the GPU last drew into it ... */
	volatile bool             gpuDrawn;     /* ... (msm client): the display needs no cache clean then */
	/*
	 * sapphire 0339: who touched the pixels last, for the cache maintenance the GPU needs (SapphireGpu.cpp,
	 * kern.sweet_gpu_flush). cpuEpoch moves whenever a processor may read or write them: every IOSurfaceLock and
	 * IOSurfaceUnlock (the two traps), a GPU buffer over them being mapped or unmapped by its program,
	 * GEM_CPU_PREP and GEM_CPU_FINI, a copy the kernel makes with the CPU. cleanEpoch is cpuEpoch as it was read
	 * before the pixels were last cleaned out of the CPU caches (dc cvac or civac: all that a processor wrote
	 * up to then is in memory), invalEpoch the same for the last clean that also invalidated (dc civac: no line
	 * of them was left in a cache). A surface is made with its memory zeroed through the kernel's mapping, so
	 * it starts touched and never cleaned.
	 */
	volatile uint32_t         cpuEpoch, cleanEpoch, invalEpoch;
	uint32_t                  wrapDelta;    /* wrapped client memory: where it starts in its first page */
	bool                      wrapped;      /* memory is a prepared range of a task (IOSurfaceWrapClient*) */
	/*
	 * sapphire 0350: lowPool, the memory is a slot of the framebuffer's scanout pool (scanoutPhys, scanoutSlot): RAM the
	 * kernel does not manage, below 4 GiB, mapped write-combined everywhere (no CPU cache holds any of it). gpuFence:
	 * the msm fence of the last submission that named the surface (SapphireGpu.cpp, under its lock), so that a scanout
	 * of it waits for whatever was drawn into it, even if submitted after its SwapEnd.
	 */
	bool                      lowPool;
	volatile uint32_t         gpuFence;
	/*
	 * sapphire 0394: gpuWriteFence, the msm fence of the last submission that wrote the surface (MSM_SUBMIT_BO_WRITE,
	 * or neither flag); settledFence, the one whose writes an IOSurfaceLock last waited for and invalidated the caches
	 * after (SapphireGpu.cpp, sapphire_gpu_surface_settle). Equal: nothing the GPU wrote is newer than the lines a
	 * processor may hold.
	 */
	volatile uint32_t         gpuWriteFence, settledFence;
	/*
	 * sapphire 0354: who made it and when (for the pool's census), and how often the window server has swapped it
	 * onto the panel and when last (SweetFramebuffer.cpp, with its fPresentLock).
	 */
	char                      creator[20];
	uint64_t                  madeAbs;
	uint32_t                  swaps;
	uint64_t                  swapAbs;
	/* 0398: shared display UBWC layout; only pool surfaces may set this. */
	bool                      ubwc;
	uint32_t                  ubwcMetaOffset, ubwcMetaStride, ubwcMetaSize;
	uint32_t                  ubwcDataOffset, ubwcDataStride;

	bool init() APPLE_KEXT_OVERRIDE { sweet_surface_counted(1); gpuSeed = 0; gpuDrawn = false; cpuEpoch = 1; cleanEpoch = 0; invalEpoch = 0; scanoutPhys = 0; scanoutSlot = -1; values = NULL; purgeable = 0; wrapDelta = 0; wrapped = false; lowPool = false; gpuFence = 0; gpuWriteFence = 0; settledFence = 0; creator[0] = 0; madeAbs = 0; swaps = 0; swapAbs = 0; ubwc = false; ubwcMetaOffset = ubwcMetaStride = ubwcMetaSize = ubwcDataOffset = ubwcDataStride = 0; return OSObject::init(); }
	void free() APPLE_KEXT_OVERRIDE;

	/* Kernel address of the pixels (mapped on first use), or 0. */
	uintptr_t kernelAddress();
};

/* Retained surface for an id, or NULL. */
SweetSurface *sweet_surface_copy(uint32_t id);

/*
 * Publish physical memory (the panel framebuffer) as a surface. Returns a
 * retained surface that stays in the id table until released by the caller
 * and every user.
 */
SweetSurface *sweet_surface_create_physical(uint64_t phys, uint32_t width,
    uint32_t height, uint32_t bytesPerRow, uint32_t pixelFormat);

/*
 * topaz scanout pool: physically contiguous low memory (below 4 GiB, unused by
 * xnu, which runs in the upper DDR bank) so the MDP can fetch a surface
 * directly. The framebuffer sets the full-screen geometry it can scan out.
 */
void sweet_scanout_set_geometry(uint32_t w1, uint32_t h1, uint32_t w2, uint32_t h2);

/*
 * sapphire 0350: the framebuffer's scanout pool (SweetFramebuffer.cpp, kern.sweet_fb_scanout). The framebuffer gives its
 * memory (slots of slot_size bytes from pa, below 4 GiB) and the size of the surfaces it can scan out of it; while
 * sweet_fb_scanout_pool_on() says so, a surface the window server makes at that size, 32-bit BGRA in one plane, gets a
 * slot. The census: slots in use and in all, surfaces given one, and refused for want of one.
 */
void sweet_scanout_pool_set(uint64_t pa, uint32_t slots, uint32_t slot_size, uint32_t width, uint32_t height);
void sweet_scanout_pool_census(uint32_t *used, uint32_t *slots, uint64_t *made, uint64_t *refused);
int sweet_fb_scanout_pool_on(void);
/* sapphire 0354: one line per slot in use (surface, its IOSurfaceName, who made it, age, swaps); bytes written */
size_t sweet_scanout_pool_describe(char *buf, size_t len);
/* sapphire 0354: whether s is still the surface published under id (not freed, not released by every user) */
bool sweet_surface_alive(uint32_t id, const SweetSurface *s);

/*
 * sapphire 0394 (SapphireGpu.cpp): before a processor reads or writes a surface the GPU wrote -- an IOSurfaceLock, of
 * any process -- wait for that submission and clean and invalidate the surface's lines; AVOID_SYNC (the lock's
 * kIOSurfaceLockAvoidSync): do not wait, settle only what is done.
 */
void sapphire_gpu_surface_settle(SweetSurface *s, bool avoidSync);

/* Send right for the surface in task (IOSurfaceLookupFromMachPort). */
mach_port_name_t sweet_surface_port_in_task(SweetSurface *surface, task_t task);

#endif /* SWEET */
#endif /* _SWEET_SURFACE_H */
