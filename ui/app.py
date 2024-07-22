import tkinter as tk 
from playsound import playsound 

# trocar o conteúdo da janela
def trocar_conteudo(janela, criar_conteudo):
    # Remove todos os widgets atuais da janela
    for widget in janela.winfo_children():
        widget.destroy()
    criar_conteudo(janela)
        
def voltar_menu(janela):
    trocar_conteudo(janela, menu_inicio)

# Janela: Contatos
def contatos(janela):

    label = tk.Label(janela, text="CONTATOS", font=("Arial", 20))
    label.pack(pady=20)
    label = tk.Label(janela, text="PROPAE:(75) 3622-4442.", font=("Arial", 15))
    label.pack(pady=15)
    label = tk.Label(janela, text="NED", font=("Arial", 15))
    label.pack(pady=15)
    label = tk.Label(janela, text="NUGET", font=("Arial", 15))
    label.pack(pady=15)

    button_voltar = tk.Button(janela, text="VOLTAR", command=lambda: voltar_menu(janela), bg='blue', fg='white', font=("Arial", 15), width=20, height=2)
    button_voltar.pack(pady=10)

# Janela: Informações
def info_menu(janela):
    label = tk.Label(janela, text="INFORMAÇÕES", font=("Arial", 20))
    label.pack(pady=20)
    label = tk.Label(janela, text="Selecione a opção que deseja:", font=("Arial", 15))
    label.pack(pady=15)
    
    button1 = tk.Button(janela, text="ACESSAR O MAPA", command=lambda: trocar_conteudo(janela, mapa_menu), bg='blue', fg='white', font=("Arial", 15), width=20, height=2)
    button1.pack(pady=10)
    
    button2 = tk.Button(janela, text="GUIA", command=lambda: trocar_conteudo(janela, guia_menu), bg='blue', fg='white', font=("Arial", 15), width=20, height=2)
    button2.pack(pady=10)
    
    button3 = tk.Button(janela, text="CONTATOS", command=lambda: trocar_conteudo(janela, contatos), bg='blue', fg='white', font=("Arial", 15), width=20, height=2)
    button3.pack(pady=10)

    button_voltar = tk.Button(janela, text="VOLTAR", command=lambda: voltar_menu(janela), bg='blue', fg='white', font=("Arial", 15), width=20, height=2)
    button_voltar.pack(pady=10)

# Janela: Mapa
def mapa_menu(janela):
    label = tk.Label(janela, text="MAPA", font=("Arial", 20))
    label.pack(pady=20)
    
    try:
        imagem_tk = tk.PhotoImage(file="imagens\Captura de tela 2024-04-30 161445.png") 
        imagem_label = tk.Label(janela, image=imagem_tk)
        imagem_label.image = imagem_tk 
        imagem_label.pack(pady=10)
    except Exception as e:
        label_erro = tk.Label(janela, text=f"Erro ao carregar a imagem tente novamente: {e}", fg="red", font=("Arial", 15))
        label_erro.pack(pady=20)
        
    label = tk.Label(janela, text="MAPA (EXEMPLO)", font=("Arial", 10))
    label.pack(pady=15)
    button_voltar = tk.Button(janela, text="VOLTAR", command=lambda: voltar_menu(janela), bg='blue', fg='white', font=("Arial", 15), width=20, height=2)
    button_voltar.pack(pady=10)

# Janela: Guia
def guia_menu(janela):
    
    label = tk.Label(janela, text="Selecione a opção que deseja:", font=("Arial", 15))
    label.pack(pady=15)
    
    label = tk.Label(janela, text="DIREÇÃO   BANHEIROS   LABORATÓRIOS", font=("Arial", 20))
    label.pack(pady=20)
    label = tk.Label(janela, text="AUDITÓRIO   BIBLIOTECA   PAV1", font=("Arial", 20))
    label.pack(pady=20)
    label = tk.Label(janela, text="GABINETES   PAV2", font=("Arial", 20))
    label.pack(pady=20)
    
    button_voltar = tk.Button(janela, text="VOLTAR", command=lambda: voltar_menu(janela), bg='blue', fg='white', font=("Arial", 15), width=20, height=2)
    button_voltar.pack(pady=10)
    
# Menu Inicial
def menu_inicio(janela):

    label = tk.Label(janela, text="MENU", font=("Arial", 20))
    label.pack(pady=20)

    button1 = tk.Button(janela, text="INFORMAÇÕES", command=lambda: trocar_conteudo(janela, info_menu), bg='blue', fg='white', font=("Arial", 15), width=20, height=2)
    button1.pack(pady=10)
    
    button2 = tk.Button(janela, text="MAPA", command=lambda: trocar_conteudo(janela, mapa_menu), bg='blue', fg='white', font=("Arial", 15), width=20, height=2)
    button2.pack(pady=10)

    button3 = tk.Button(janela, text="GUIA", command=lambda: trocar_conteudo(janela, guia_menu), bg='blue', fg='white', font=("Arial", 15), width=20, height=2)
    button3.pack(pady=10)

def splash_screen():
    playsound("MENU.mp3")
    splash = tk.Tk()
    splash.title("ROBÔ RAUL")
    splash.geometry("1350x1350")
    splash.configure(background="blue")

    label = tk.Label(splash, text="ROBÔ RAUL", background="blue", font=("Arial", 25), fg="#FFFFFF")
    label.pack(expand=True)

    splash.after(5000, lambda: iniciar_menu_principal(splash))


    splash.mainloop()

# iniciar o menu principal
def iniciar_menu_principal(splash):
    playsound("audio.mp3")
    splash.destroy() 
    janela_principal = tk.Tk()  
    janela_principal.title("Menu Principal")
    janela_principal.geometry("1350x1350")
    menu_inicio(janela_principal) 
    janela_principal.mainloop()  

if __name__ == "__main__":
    splash_screen()

