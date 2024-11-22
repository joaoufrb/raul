import tkinter as tkk
from tkinter import ttk
import speech_recognition as sr
import pygame
import os
import pickle
import cv2
import mediapipe as mp
import pyautogui
import unittest
import threading
import time

def trocar_conteudo(janela, criar_conteudo):
    for widget in janela.winfo_children():
        widget.destroy()
    criar_conteudo(janela)

def voltar_menu(janela):
    trocar_conteudo(janela, menu_inicio)

def criar_cabecalho(janela, toggle_menu):
    hed_frame = tkk.Frame(janela, bg="#f1ab05", highlightbackground='White', highlightthickness=1)

    #menu hambúrguer
    toggle_btn = tkk.Button(hed_frame, text="☰", bg="#f1ab05", fg="black", font=("Arial", 20), bd=0, 
                        activebackground='#f1ab05', activeforeground='white', command=lambda: toggle_menu(janela, toggle_btn))

    title_lb = tkk.Label(hed_frame, text='RAUL', bg='#f1ab05', fg='black', font=('Arial', 20, 'bold'))

    toggle_btn.pack(side=tkk.LEFT)
    title_lb.pack(side=tkk.LEFT, padx=10)

    hed_frame.pack(side=tkk.TOP, fill=tkk.X)
    hed_frame.pack_propagate(False)
    hed_frame.configure(height=50)
    
    return toggle_btn

def toggle_menu(janela, toggle_btn):
    def collapse_toggle_menu(toggle_menu_fm):
        toggle_menu_fm.destroy()
        toggle_btn.config(text='☰')
        toggle_btn.config(command=lambda: toggle_menu(janela, toggle_btn))

    toggle_menu_fm = tkk.Frame(janela, bg='#f1ab05')

    guia_btn = tkk.Button(toggle_menu_fm, text='GUIA', font=('Arial', 15, 'bold'), bd=0, bg='#f1ab05', fg='black',
                activebackground='#f1ab05', activeforeground='white',
                command=lambda: trocar_conteudo(janela, guia_menu))
    guia_btn.place(x=20, y=20)

    info_btn = tkk.Button(toggle_menu_fm, text='INFORMAÇÕES', font=('Arial', 15, 'bold'), bd=0, bg='#f1ab05', fg='black',
                        activebackground='#f1ab05', activeforeground='white',
                        command=lambda: trocar_conteudo(janela, info_menu))
    info_btn.place(x=20, y=60)
    
    mapa_btn = tkk.Button(toggle_menu_fm, text='MAPA', font=('Arial', 15, 'bold'), bd=0, bg='#f1ab05', fg='black',
                        activebackground='#f1ab05', activeforeground='white',
                        command=lambda: trocar_conteudo(janela, mapa_menu))
    mapa_btn.place(x=20, y=100)
    
    contatos_btn = tkk.Button(toggle_menu_fm, text='CONTATOS', font=('Arial', 15, 'bold'), bd=0, bg='#f1ab05', fg='black',
                        activebackground='#f1ab05', activeforeground='white',
                        command=lambda: trocar_conteudo(janela, contatos))
    contatos_btn.place(x=20, y=140)


    window_height = janela.winfo_height()
    toggle_menu_fm.place(x=0, y=50, height=window_height, width=200)

    #botão para fechar menu
    toggle_btn.config(text='X')
    toggle_btn.config(command=lambda: collapse_toggle_menu(toggle_menu_fm))
    
def reconhecer_comando_voz(janela):
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Ajustando o microfone...")
        recognizer.adjust_for_ambient_noise(source)
        print("Pode falar agora...")
        audio = recognizer.listen(source)
    try:
        comando = recognizer.recognize_google(audio, language="pt-BR")
        print(f"Você disse: {comando}")
        if "guia" in comando.lower():
            trocar_conteudo(janela, guia_menu)
        if "mapa" in comando.lower():
            trocar_conteudo(janela, mapa_menu)
    except sr.UnknownValueError:
        print("Não entendi o comando.")
    except sr.RequestError:
        print("Não consegui conectar ao serviço de reconhecimento de voz.")

def iniciar_reconhecimento_voz(janela):
    hed_frame = tkk.Frame(janela, bg="#f1ab06", highlightbackground='#f1ab06', highlightthickness=1)
    reconhecimento_voz_btn = tkk.Button(janela, text="Falar", command=lambda: reconhecer_comando_voz(janela), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'))
    reconhecimento_voz_btn.pack(side=tkk.LEFT)
    
    hed_frame.pack(side=tkk.TOP, fill=tkk.X)
    hed_frame.pack_propagate(True)
    hed_frame.configure(height=60)
 
def criar_tabela(janela):
    tabela_frame = tkk.Frame(janela)
    tabela_frame.pack(pady=20, fill=tkk.BOTH, expand=True)

    #Barras de rolagem
    vsb = tkk.Scrollbar(tabela_frame, orient="vertical")
    vsb.pack(side='right', fill='y')

    hsb = tkk.Scrollbar(tabela_frame, orient="horizontal")
    hsb.pack(side='bottom', fill='x')

    style = ttk.Style()
    style.configure("Treeview",
                    font=('Arial', 12))  
    style.configure("Treeview.Heading",
                    font=('Arial', 14, 'bold'))  

    tree = ttk.Treeview(tabela_frame, columns=("Departamento", "Telefone", "E-mail"), show='headings', yscrollcommand=vsb.set, xscrollcommand=hsb.set)
    tree.heading("Departamento", text="Departamento")
    tree.heading("Telefone", text="Telefone")
    tree.heading("E-mail", text="E-mail")
    tree.column("Departamento", width=150, anchor='center')
    tree.column("Telefone", width=150, anchor='center')
    tree.column("E-mail", width=150, anchor='center')
    tree.pack(side='left', fill=tkk.BOTH, expand=True)
    
    vsb.config(command=tree.yview)
    hsb.config(command=tree.xview)

    #Dados da tabela
    dados = [
        ("PROPAE", "(75) 3622-4442", "cetens@propae.ufrb.edu.br"),
        ("NED", "(75) 3622-5555", "ned@exemplo.com"),
        ("NUGET", "(75) 3622-6666", "nuget@exemplo.com")
    ]
    
    for departamento, telefone, email in dados:
        tree.insert("", 'end', values=(departamento, telefone, email))
        
#Atalho Ctrl+C para abrir os contatos
def contatos(janela):

    criar_cabecalho(janela, toggle_menu) 

    label = tkk.Label(janela, text="CONTATOS", font=("Arial", 20, 'bold'))
    label.pack(pady=20)

    criar_tabela(janela)

    button_voltar = tkk.Button(janela, text="VOLTAR", command=lambda: voltar_menu(janela), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20, height=2)
    button_voltar.pack(pady=10)

def info_menu(janela):
    criar_cabecalho(janela, toggle_menu) 
    
    label = tkk.Label(janela, text="INFORMAÇÕES", font=("Arial", 20))
    label.pack(pady=20)
    label = tkk.Label(janela, text="Selecione a opção que deseja:", font=("Arial", 15))
    label.pack(pady=15)

    button1 = tkk.Button(janela, text="ACESSAR O MAPA", command=lambda: trocar_conteudo(janela, mapa_menu), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20, height=2)

    button1.pack(pady=10)

    button2 = tkk.Button(janela, text="GUIA", command=lambda: trocar_conteudo(janela, guia_menu), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20, height=2)
    button2.pack(pady=10)

    button3 = tkk.Button(janela, text="CONTATOS", command=lambda: trocar_conteudo(janela, contatos), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20, height=2)
    button3.pack(pady=10)

    button_voltar = tkk.Button(janela, text="VOLTAR", command=lambda: voltar_menu(janela), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20, height=2)
    button_voltar.pack(pady=10)
    
#Atalho Ctrl+M para abrir o mapa
def mapa_menu(janela):
    criar_cabecalho(janela, toggle_menu) 
    
    label = tkk.Label(janela, text="MAPA", font=("Arial", 20))
    label.pack(pady=20)

    try:
        imagem_tk = tkk.PhotoImage(file="imagens/Captura_de_tela_2024-04-30_161445.png") 
        imagem_label = tkk.Label(janela, image=imagem_tk)
        imagem_label.image = imagem_tk  
        imagem_label.pack(pady=10)
    except Exception as e:
        label_erro = tkk.Label(janela, text=f"Erro ao carregar a imagem tente novamente: {e}", fg="red", font=("Arial", 15))
        label_erro.pack(pady=20)

    label = tkk.Label(janela, text="MAPA (EXEMPLO)", font=("Arial", 10))
    label.pack(pady=15)
    button_voltar = tkk.Button(janela, text="VOLTAR", command=lambda: voltar_menu(janela), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20, height=2)
    button_voltar.pack(pady=10)
    
    janela.bind_all("<Escape>", lambda event: voltar_menu(janela))#Atalho Esc para voltar ao menu
    
#Atalho Ctrl+G para abrir o guia
def guia_menu(janela):
    criar_cabecalho(janela, toggle_menu) 
    
    label = tkk.Label(janela, text="Selecione a opção que deseja:", font=("Arial", 15))
    label.pack(pady=15)

    label = tkk.Label(janela, text="DIREÇÃO   BANHEIROS   LABORATÓRIOS", font=("Arial", 20))
    label.pack(pady=20)
    label = tkk.Label(janela, text="AUDITÓRIO   BIBLIOTECA   PAV1", font=("Arial", 20))
    label.pack(pady=20)
    label = tkk.Label(janela, text="GABINETES   PAV2", font=("Arial", 20))
    label.pack(pady=20)

    button_voltar = tkk.Button(janela, text="VOLTAR", command=lambda: voltar_menu(janela), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20, height=2)
    button_voltar.pack(pady=10)

def menu_inicio(janela):
    criar_cabecalho(janela, toggle_menu) 
     
    label = tkk.Label(janela, text="MENU", font=("Arial", 15))
    label.pack(pady=15)

    button1 = tkk.Button(janela, text="MAPA", command=lambda: trocar_conteudo(janela, mapa_menu), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20, height=2)
    button1.pack(pady=10)

    button2 = tkk.Button(janela, text="GUIA", command=lambda: trocar_conteudo(janela, guia_menu), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20, height=2)
    button2.pack(pady=10)

    button3 = tkk.Button(janela, text="CONTATOS", command=lambda: trocar_conteudo(janela, contatos), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20, height=2)
    button3.pack(pady=10)
    
    janela.bind_all("<Control-m>", lambda event: trocar_conteudo(janela, mapa_menu))#Atalho Ctrl+M para abrir o mapa
    janela.bind_all("<Control-g>", lambda event: trocar_conteudo(janela, guia_menu))#Atalho Ctrl+G para abrir o guia
    janela.bind_all("<Control-c>", lambda event: trocar_conteudo(janela, contatos))#Atalho Ctrl+C para abrir os contatos
    janela.bind_all("<Escape>", lambda event: voltar_menu(janela))#Atalho Esc para voltar ao menu
    janela.bind.all("<Control-f>", lambda event: reconhecer_comando_voz(janela))
    #adicionar atalho para ativar o microfone

    iniciar_reconhecimento_voz(janela)
    
def boas_vindas(janela):
  
    label = tkk.Label(janela, text='Olá, eu sou o Raul, seu robô guia!\n'
    'Estou aqui para te ajudar a encontrar qualquer lugar dentro do CETENS.\n'
    'Precisa de ajuda para localizar uma sala, setor ou serviço?', font=("Arial", 20))
    label.pack(pady=20)

    #adicionar imagem de microfone
    
    #adicionar aúdio 
    pygame.mixer.init()  
    pygame.mixer.music.load('/home/aluno/Downloads/Tainara/Interface/apresentacao.mp3')  
    pygame.mixer.music.play()
    
    button1 = tkk.Button(janela, text="SALA", command=lambda: trocar_conteudo(janela, guia_menu), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20,     height=2)
    button1.pack(pady=10)
    button2 = tkk.Button(janela, text="SETOR", command=lambda: trocar_conteudo(janela, guia_menu), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20,     height=2)
    button2.pack(pady=10)
    button3 = tkk.Button(janela, text="CONTINUAR", command=lambda: trocar_conteudo(janela, menu_inicio), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'), width=20,     height=2)
    button3.pack(pady=10)
  
    reconhecimento_voz_btn = tkk.Button(janela, text="Falar", command=lambda: reconhecer_comando_voz(janela), bg='#f1ab06', fg='black', font=("Arial", 15, 'bold'))
    reconhecimento_voz_btn.pack(side=tkk.LEFT)

    #(opção da fala + comando de voz)
    
    #adicionar audio para ativar ou desativar
    pygame.mixer.init()  
    pygame.mixer.music.load("apresentacao.mp3")  
    pygame.mixer.music.play()

    iniciar_reconhecimento_voz(janela)
    
    janela_boas_vindas.mainloop()

def iniciar_menu_principal(splash):
    splash.destroy()
    janela_principal = tkk.Tk()  
    janela_principal.title("Menu Principal")
    janela_principal.geometry("1350x700")
    
    boas_vindas(janela_principal)  
    
    janela_principal.mainloop()

def splash_screen():
    splash = tkk.Tk()
    splash.title("ROBÔ RAUL")
    splash.geometry("1350x700")
    splash.configure(background="white")

    splash.after(5000, lambda: iniciar_menu_principal(splash))

    splash.mainloop()

if __name__ == "__main__":
    splash_screen()
