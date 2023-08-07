# Copyright 2023, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

ANGLER_NAMESPACE = "/angler"

# Topic names
TOPIC_ARM = f"{ANGLER_NAMESPACE}/arm"

# Blackboard keys
BB_ARMED = "armed"
BB_PASSTHROUGH_REQUEST_RESPONSE = "passthrough_request_response"
BB_BLUE_ARMING_REQUEST_RESPONSE = "blue_arming_request_response"
BB_ANGLER_ARMING_REQUEST_RESPONSE = "angler_arming_request_response"

# Service names
SRV_ENABLE_PASSTHROUGH = "/blue/cmd/enable_passthrough"
SRV_ARM_BLUE = "/blue/cmd/arm"
SRV_ARM_ANGLER = "/angler/cmd/arm"
