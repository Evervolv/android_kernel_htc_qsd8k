/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __KGSL_PWRSCALE_H
#define __KGSL_PWRSCALE_H

struct kgsl_pwrscale;

struct kgsl_pwrscale_policy  {
	const char *name;
	int (*init)(struct kgsl_device *device,
		struct kgsl_pwrscale *pwrscale);
	void (*close)(struct kgsl_device *device,
		struct kgsl_pwrscale *pwrscale);
	void (*idle)(struct kgsl_device *device,
		struct kgsl_pwrscale *pwrscale);
	void (*busy)(struct kgsl_device *device,
		struct kgsl_pwrscale *pwrscale);
	void (*sleep)(struct kgsl_device *device,
		struct kgsl_pwrscale *pwrscale);
	void (*wake)(struct kgsl_device *device,
		struct kgsl_pwrscale *pwrscale);
};

struct kgsl_pwrscale {
	struct kgsl_pwrscale_policy *policy;
	struct kobject kobj;
	struct completion kobj_unregister;
	void *priv;
};

struct kgsl_pwrscale_policy_attribute {
	struct attribute attr;
	ssize_t (*show)(struct kgsl_device *device,
			struct kgsl_pwrscale *pwrscale, char *buf);
	ssize_t (*store)(struct kgsl_device *device,
			 struct kgsl_pwrscale *pwrscale, const char *buf,
			 size_t count);
};

#define PWRSCALE_POLICY_ATTR(_name, _mode, _show, _store)          \
	struct kgsl_pwrscale_policy_attribute policy_attr_##_name = \
		__ATTR(_name, _mode, _show, _store)

int kgsl_pwrscale_init(struct kgsl_device *device);
void kgsl_pwrscale_close(struct kgsl_device *device);
int kgsl_pwrscale_attach_policy(struct kgsl_device *device,
	struct kgsl_pwrscale_policy *policy);
void kgsl_pwrscale_detach_policy(struct kgsl_device *device);

void kgsl_pwrscale_idle(struct kgsl_device *device);
void kgsl_pwrscale_busy(struct kgsl_device *device);
void kgsl_pwrscale_sleep(struct kgsl_device *device);
void kgsl_pwrscale_wake(struct kgsl_device *device);

int kgsl_pwrscale_policy_add_files(struct kgsl_device *device,
				   struct kgsl_pwrscale *pwrscale,
				   struct attribute_group *attr_group);

void kgsl_pwrscale_policy_remove_files(struct kgsl_device *device,
				       struct kgsl_pwrscale *pwrscale,
				       struct attribute_group *attr_group);
#endif
