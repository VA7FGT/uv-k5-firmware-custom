/* Copyright 2024 kamilsss655
 * https://github.com/kamilsss655
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include "crypto.h"
#include "external/chacha/chacha.h"

// Used for both encryption and decryption
void CRYPTO_Crypt(void *input, int input_len, void *output, void *nonce, const void *key, int key_len)
{
	struct chacha_ctx ctx;

	unsigned char keystream[CHACHA_BLOCKLEN];

	memset(&ctx, 0, sizeof(ctx));
	chacha_keysetup(&ctx, key, key_len);

	// init keystream and generate key
	memset(keystream, 0, sizeof(keystream));
	chacha_ivsetup(&ctx, nonce, NULL);
	chacha_encrypt_bytes(&ctx, keystream, keystream,sizeof(keystream));

	// crypt data, only works for input_len <= 32
	for (uint8_t i = 0; i < input_len; i++) {
		((unsigned char *)output)[i] =
			((unsigned char *)input)[i] ^ keystream[32 + i];
	}
}

			   