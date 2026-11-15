import pygame as pg
import sys
class Interface:
    def __init__(self,width,height):
        self.width=width
        self.height=height
        pg.init()
        self.screen=pg.display.set_mode((self.width,self.height))
        pg.display.set_caption("Mobile Robot Simulation")
    def initial_screen(self):
        self.screen.fill((255,255,255))
        self.screen.blit(pg.font.SysFont("Arial",30).render("Press any key to start",True,(0,0,0)),(self.width//2-150,self.height//2))
        pg.display.update()
        
        execute=True

        while execute:



interface=Interface(700,400)
interface.initial_screen()
