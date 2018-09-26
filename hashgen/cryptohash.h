/*
* Authod: Marius Bancila (http://mariusbancila.ro)
* License: Code Project Open License
* Disclaimer: The software is provided "as-is". No claim of suitability, guarantee, or any warranty whatsoever is provided.
* Published: 19.09.2012
*/

//#pragma once
#ifndef CRYPTOHASH_H
#define CRYPTOHASH_H

#include <Windows.h>
#include <WinCrypt.h>
#pragma comment(lib, "Advapi32.lib")

#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <fstream>

namespace crypto
{
   typedef std::vector<unsigned char> hash_t;

   class string_utils
   {
   public:
      static std::string hextostr(std::vector<unsigned char> const & hexval, bool uppercase = false)
      {
         std::stringstream sstr;

         if(!hexval.empty())
         {
            sstr.setf(std::ios_base::hex, std::ios::basefield);
            if(uppercase)
               sstr.setf(std::ios_base::uppercase);
            sstr.fill('0');
            for (size_t i = 0; i<hexval.size(); i++)
            {
               sstr << std::setw(2) << (unsigned int)(unsigned char)(hexval[i]);
            }
         }

         return sstr.str();
      }
   };

   struct errorinfo_t
   {
      DWORD errorCode;
      std::string errorMessage;

      errorinfo_t(DWORD code = 0, std::string const &message = ""):
      errorCode(code),errorMessage(message)
      {}
   };

   template <ALG_ID algorithm>
   class cryptohash_t
   {
   public:

      cryptohash_t(void): m_hCryptProv(NULL), m_hHash(NULL)
      {
      }

      ~cryptohash_t(void)
      {
         if(m_hHash != NULL)
         {
            ::CryptDestroyHash(m_hHash); 
         }

         if(m_hCryptProv != NULL)
         {
            ::CryptReleaseContext(m_hCryptProv, 0);
         }
      }

      bool begin()
      {
         m_lasterror = errorinfo_t();

         if(m_hCryptProv != NULL && m_hHash != NULL)
         {
            m_lasterror = errorinfo_t(0, "Cryptographic provider already acquired!");
            return false;
         }

         m_digest.clear();

         if(!::CryptAcquireContext(
            &m_hCryptProv, 
            NULL, 
            NULL, 
            PROV_RSA_FULL, 
            CRYPT_VERIFYCONTEXT | CRYPT_MACHINE_KEYSET)) 
         {
            m_lasterror = errorinfo_t(GetLastError(), "Failed to acquire cryptographic context.");
            return false;
         }

         if(!::CryptCreateHash(
            m_hCryptProv, 
            algorithm,
            0, 
            0, 
            &m_hHash))
         {
            m_lasterror = errorinfo_t(GetLastError(), "Failed to acquire cryptographic context.");

            ::CryptReleaseContext(m_hCryptProv, 0); 
            m_hCryptProv = NULL;
            return false;
         }

         return true;
      }

      bool update(unsigned char* const buffer, size_t size)
      {
         m_lasterror = errorinfo_t();

         if(m_hCryptProv == NULL || m_hHash == NULL)
         {
            m_lasterror = errorinfo_t(0, "Cryptographic provider not acquired!");
            return false;
         }

         if(!::CryptHashData(
            m_hHash, 
            buffer, 
            size, 
            0))
         {
            m_lasterror = errorinfo_t(GetLastError(), "Updating hash failed!");
            return false;
         }

         return true;
      }

      bool finalize()
      {
         m_lasterror = errorinfo_t();

         if(m_hCryptProv == NULL || m_hHash == NULL)
         {
            m_lasterror = errorinfo_t(0, "Cryptographic provider not acquired!");
            return false;
         }

         bool success = false;
         DWORD dwHashLen = 0;
         DWORD dwHashLenSize = sizeof(dwHashLen);
         if(::CryptGetHashParam(
            m_hHash,
            HP_HASHSIZE,
            (BYTE*)&dwHashLen,
            &dwHashLenSize,
            0))
         {
            m_digest.resize(dwHashLen, 0);

            if(dwHashLen > 0)
            {
               if(::CryptGetHashParam(
                  m_hHash, 
                  HP_HASHVAL, 
                  &m_digest[0], 
                  &dwHashLen, 
                  0))
               {
                  success = true;
               }
               else
               {
                  m_lasterror = errorinfo_t(GetLastError(), "Failed retrieving hash value!");
               }
            }
         }
         else
         {
            m_lasterror = errorinfo_t(GetLastError(), "Failed computing hash length!");
         }

         if(m_hHash != NULL)
         {
            ::CryptDestroyHash(m_hHash); 
            m_hHash = NULL;
         }

         if(m_hCryptProv != NULL)
         {
            ::CryptReleaseContext(m_hCryptProv, 0);
            m_hCryptProv = NULL;
         }

         return success;
      }

      hash_t digest() const {return m_digest;}
      std::string hexdigest(bool uppercase = false) const {return string_utils::hextostr(m_digest, uppercase);}
      errorinfo_t lasterror() const {return m_lasterror;}

   private:
      errorinfo_t m_lasterror;
      HCRYPTPROV m_hCryptProv; 
      HCRYPTHASH m_hHash; 
      hash_t m_digest;
   };

   template <ALG_ID algorithm>
   class cryptohash_helper_t
   {
      errorinfo_t m_lasterror;
   public:
      hash_t digesttext(std::string const& text)
      {
         cryptohash_t<algorithm> mdx;

         if(mdx.begin())
         {
            if(mdx.update((unsigned char*)(text.c_str()), text.length()))
            {
               mdx.finalize();
            }
         }

         m_lasterror = mdx.lasterror();

         return mdx.digest();
      }

      std::string hexdigesttext(std::string const& text, bool uppercase = false)
      {
         return string_utils::hextostr(digesttext(text), uppercase);
      }

      hash_t digestfile(std::string const& filename)
      {
         cryptohash_t<algorithm> mdx;

         std::ifstream file;
         file.open(filename.c_str(), std::ios::binary|std::ios::in);
         if(file.is_open())
         {
            file.seekg(0, std::ios::end);
            size_t filesize = file.tellg();
            file.seekg(0, std::ios::beg);

            if(filesize > 0)
            {
               if(mdx.begin())
               {
                  size_t total = 0;
                  unsigned char chunk[8*1024];
                  do
                  {
                     size_t left = (filesize - total) > sizeof(chunk) ? sizeof(chunk) : (filesize - total);
                     file.read(reinterpret_cast<char*>(chunk), left);

                     size_t totalread = file.gcount();
                     total += totalread;

                     if(totalread > 0)
                     {
                        if(!mdx.update(chunk, totalread))
                           break;
                     }
                  }
                  while(total < filesize);

                  if(total == filesize)
                  {
                     mdx.finalize();
                  }
               }

               m_lasterror = mdx.lasterror();
            }

            file.close();
         }
         else
         {
            m_lasterror = errorinfo_t(0, filename + " could not be opened");
         }

         return mdx.digest();
      }

      std::string hexdigestfile(std::string const& filename, bool uppercase = false)
      {
         return string_utils::hextostr(digestfile(filename), uppercase); 
      }

      errorinfo_t lasterror() const {return m_lasterror;}
   };

   typedef cryptohash_t<CALG_MD2> md2_t;
   typedef cryptohash_t<CALG_MD4> md4_t;
   typedef cryptohash_t<CALG_MD5> md5_t;
   typedef cryptohash_t<CALG_SHA1> sha1_t;

   typedef cryptohash_helper_t<CALG_MD2> md2_helper_t;
   typedef cryptohash_helper_t<CALG_MD4> md4_helper_t;
   typedef cryptohash_helper_t<CALG_MD5> md5_helper_t;
   typedef cryptohash_helper_t<CALG_SHA1> sha1_helper_t;
}
#endif //CRYPTOHASH_H
