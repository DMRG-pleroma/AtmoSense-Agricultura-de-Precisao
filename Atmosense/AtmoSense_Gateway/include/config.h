#ifndef CONFIG_H
#define CONFIG_H

// =====================================================================
// CREDENCIAIS DE CONEXÃO E INTEGRAÇÃO (TEMPLATE)
// =====================================================================
// ATENÇÃO: Os dados abaixo utilizam placeholders por segurança.
// Substitua pelas configurações reais da sua rede e banco de dados.

#define WIFI_SSID "NOME_DA_SUA_REDE_WIFI"
#define WIFI_PASS "SUA_SENHA_WIFI"

// Endpoint da API REST (Exemplo utilizando o Supabase como no projeto)
#define API_URL   "https://sua-api-rest.supabase.co/rest/v1/atmosense"

// Token de Acesso JWT da sua API de Produção
#define API_KEY   "Bearer SEU_JWT_SUPABASE_TOKEN"

#endif // CONFIG_H