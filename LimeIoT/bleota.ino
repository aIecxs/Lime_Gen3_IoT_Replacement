/*
  Create a BLE server with BLE OTA service that implement https://components.espressif.com/components/espressif/ble_ota with security
  
  The service advertises itself as: 00008018-0000-1000-8000-00805f9b34fb
  If any of this is defined:
  MODEL
  SERIAL_NUM
  FW_VERSION
  HW_VERSION
  MANUFACTURER
  the DIS service is added

  The flow of creating the BLE server is:
  1. Create a BLE Server
  2. Add a BLE OTA Service with security enabled
  2. Add the public key (rsa_key.pub content)
  3. Add a DIS Service
  4. Start the service.
  5. Start advertising.
  6. Process the update
  
  The .bin file must be signed and the public key must be included in the sketch.
  The signing process is the following:
  generate the private key (keep this secret):
    - openssl genrsa -out priv_key.pem 2048

  and the corresponding public key:
    - openssl rsa -in priv_key.pem -pubout > rsa_key.pub

  export the compiled sketch or SPIFFS to get the bin file
    
  sign the file with SHA256 hash with the private key:
    - openssl dgst -sign priv_key.pem -keyform PEM -sha256 -out signature.sign -binary ota.ino.bin

  throw it all in one file
    - cat ota.ino.bin signature.sign > ota.bin

  use this file to perform the update

*/

// Enable Security
#define BLEOTA_SET_SECURITY_AUTH
/*
   Legacy Force NimBLE-Arduino (h2zero) BLE stack on core ≥ 3.3.0+ (External dependency)

   you may use that modified fork instead of the original BLEOTA Library
   Credits: @gb88
 
   github.com/aIecxs/BLEOTA/tree/nimble -> download zip file ->
   github.com/aIecxs/BLEOTA/archive/refs/heads/nimble.zip

   Install the Library in Arduino IDE 2 -> Menu ->
   Sketch -> Include Library -> Add .ZIP Library... -> BLEOTA-nimble.zip
*/
#include <BLEOTA.h>
#ifndef BLEOTA_USE_NIMBLE
  #error "#define BLEOTA_USE_NIMBLE -> hardcode straight in {otherLibrariesFolders}/BLEOTA/src -> NimBLEOTA.h library header!"
#endif


// Auto Generating persistent RSA 2048-bit key pair files
#include <LittleFS.h>
#include <esp_task_wdt.h>
#include <mbedtls/pk.h>
#include <mbedtls/rsa.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>


#define MODEL "Lime-S Gen3.0"
//#define SERIAL_NUM
#define FW_VERSION "1.0.1"
#define HW_VERSION "LSW6G-Lime2.0-181117RC"
#define MANUFACTURER "Espressif"


char* pub_key = nullptr;
inline constexpr const char* pubKeyFile = "/rsa_key.pub";
inline constexpr const char* privKeyFile = "/priv_key.pem";

BLEOTAClass BLEOTA;
bool rsaKeys = false;


// BLE OTA callbacks
class OTACallbacks : public BLEOTACallbacks {
  void beforeStartSPIFFS() {
    isUpgrading = true;
  }
  void beforeStartOTA() {
    isUpgrading = true;
  }
  void afterStop() {
    isUpgrading = false;
  }
  void afterAbort() {
    isUpgrading = false;
  }
};


void initBLEOTA() {

  // Add OTA Service with security
  BLEOTA.begin(pServer, true);
  // Add pub key
  BLEOTA.setKey(pub_key, strlen(pub_key));
#ifdef MODEL
  BLEOTA.setModel(MODEL);
#endif
#ifdef SERIAL_NUM
  BLEOTA.setSerialNumber(SERIAL_NUM);
#endif
#ifdef FW_VERSION
  BLEOTA.setFWVersion(FW_VERSION);
#endif
#ifdef HW_VERSION
  BLEOTA.setHWVersion(HW_VERSION);
#endif
#ifdef MANUFACTURER
  BLEOTA.setManufactuer(MANUFACTURER);
#endif

  BLEOTA.setCallbacks(new OTACallbacks());
  BLEOTA.init();

#ifdef FW_VERSION
  Serial.print("Firmware Version: ");
  Serial.println(FW_VERSION);
#endif

}


// Add the public key (rsa_key.pub content)
char* loadPemFromLittleFS(const char* keyfile) {
  // open public PEM file
  File pubFile = LittleFS.open(keyfile, "r");
  if (!pubFile) {
    Serial.print("LittleFS: cannot access '");
    Serial.print(keyfile);
    Serial.println("': No such file or directory");
    return nullptr;
  }
  size_t pem_len = pubFile.size();
  unsigned char *pubKey = (unsigned char*)malloc(pem_len + 1);
  if (!pubKey) {
    pubFile.close();
    Serial.println("Failed to allocate memory for public key");
    return nullptr;
  }
  pubFile.read(pubKey, pem_len);
  pubFile.close();
  pubKey[pem_len] = '\0';
  return (char*)pubKey;
}


// generate RSA 2048-bit private.pem + public.pem key pair files
void generateKeys() {
  if (rsaKeys) return; // already checked
  bool keysValid = false;

  // check first 10 bytes match "^-----BEGIN"
  if (LittleFS.exists(privKeyFile) && LittleFS.exists(pubKeyFile)) {
    File privFile = LittleFS.open(privKeyFile, "r");
    File pubFile  = LittleFS.open(pubKeyFile, "r");
    if (privFile && pubFile) {
      char header[11] = {0};
      privFile.readBytes(header, 10);
      if (strncmp(header, "-----BEGIN", 10) == 0) {
        pubFile.readBytes(header, 10);
        if (strncmp(header, "-----BEGIN", 10) == 0) {
          keysValid = true;
        }
      }
    }
    privFile.close();
    pubFile.close();
  }

  if (!keysValid) {
    Serial.println("Generating RSA 2048-bit key pair...");

    // Initialize Mbed TLS structures
    mbedtls_pk_context pk;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_entropy_context entropy;
    const char *pers = "rsa_gen";

    mbedtls_pk_init(&pk);
    mbedtls_ctr_drbg_init(&ctr_drbg);
    mbedtls_entropy_init(&entropy);

    if (mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy, (const unsigned char*)pers, strlen(pers)) != 0) {
      Serial.println("DRBG seed failed");
      return;
    }

    if (mbedtls_pk_setup(&pk, mbedtls_pk_info_from_type(MBEDTLS_PK_RSA)) != 0) {
      Serial.println("PK setup failed");
      return;
    }

    // disable watchdog for this task
    esp_task_wdt_delete(NULL);

    if (mbedtls_rsa_gen_key(mbedtls_pk_rsa(pk), mbedtls_ctr_drbg_random, &ctr_drbg, 2048, 65537) != 0) {
      Serial.println("RSA key generation failed");
      esp_task_wdt_add(NULL);  // re-enable before returning
      return;
    }
    // re-enable watchdog
    esp_task_wdt_add(NULL);

    // write private key to PEM
    unsigned char privPem[1792];
    if (mbedtls_pk_write_key_pem(&pk, privPem, sizeof(privPem)) != 0) {
      Serial.println("Private key PEM export failed");
      return;
    }
    File fPriv = LittleFS.open(privKeyFile, "w");
    fPriv.write(privPem, strlen((char*)privPem));
    fPriv.close();

    // write public key to PEM
    unsigned char pubPem[512];
    if (mbedtls_pk_write_pubkey_pem(&pk, pubPem, sizeof(pubPem)) != 0) {
      Serial.println("Public key PEM export failed");
      return;
    }
    File fPub = LittleFS.open(pubKeyFile, "w");
    fPub.write(pubPem, strlen((char*)pubPem));
    fPub.close();

    mbedtls_pk_free(&pk);
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);

    Serial.println("RSA key pair generated.");
  }

  rsaKeys = true;
}
