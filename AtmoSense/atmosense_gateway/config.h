#ifndef CONFIG_H
#define CONFIG_H

// =====================================================================
// CREDENCIAIS DE CONEXÃO E INTEGRAÇÃO
// =====================================================================
#define WIFI_SSID "NOME_DA_SUA_REDE_WIFI"
#define WIFI_PASS "SUA_SENHA_WIFI"

// Endpoint da API REST (Exemplo utilizando o Supabase)
#define API_URL   "https://sua-api-rest.supabase.co/rest/v1/atmosense"

// Token de Acesso JWT da sua API de Produção
#define API_KEY   "Bearer SEU_JWT_SUPABASE_TOKEN"

// =====================================================================
// COORDENADAS DA PROPRIEDADE (Horta AtmoSense)
// =====================================================================
#define LATITUDE  "-25.40"
#define LONGITUDE "-52.41"

#endif // CONFIG_H
